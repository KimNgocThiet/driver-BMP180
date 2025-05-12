#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>

#define DRIVER_NAME "bmp180_driver"
#define DEVICE_NAME "bmp180"
#define CLASS_NAME "bmp180"

#define BMP180_ADDR 0x77

/* BMP180 registers */
#define BMP180_REG_CALIBRATION 0xAA
#define BMP180_REG_CTRL_MEAS   0xF4
#define BMP180_REG_OUT_MSB     0xF6

/* BMP180 control values */
#define BMP180_CMD_TEMP        0x2E
#define BMP180_CMD_PRESSURE_OSS0 0x34
#define BMP180_CMD_PRESSURE_OSS1 0x74
#define BMP180_CMD_PRESSURE_OSS2 0xB4
#define BMP180_CMD_PRESSURE_OSS3 0xF4

/* Conversion times */
#define BMP180_TEMP_CONVERSION_TIME_MS 5
#define BMP180_PRESSURE_CONVERSION_TIME_MS(oss) (5 + (3 << (oss)))

/* Calibration data structure */
struct bmp180_calibration {
    s16 AC1, AC2, AC3;
    u16 AC4, AC5, AC6;
    s16 B1, B2;
    s16 MB, MC, MD;
};

struct bmp180_data {
    struct i2c_client *client;
    struct cdev cdev;
    struct bmp180_calibration calib;
    dev_t devno;
    struct class *class;
    struct device *device;
    int oversampling_setting;
    struct mutex lock;  // Mutex for protecting device access
};

/* ioctl commands */
#define BMP180_IOC_MAGIC 'b'
#define BMP180_IOC_SET_OSS _IOW(BMP180_IOC_MAGIC, 1, int)
#define BMP180_IOC_GET_OSS _IOR(BMP180_IOC_MAGIC, 2, int)
#define BMP180_IOC_RESET    _IO(BMP180_IOC_MAGIC, 3)

static struct bmp180_data *bmp180_dev;

static int bmp180_read_calibration(struct i2c_client *client, struct bmp180_calibration *calib)
{
    u8 buf[22];
    int ret;

    ret = i2c_smbus_read_i2c_block_data(client, BMP180_REG_CALIBRATION, sizeof(buf), buf);
    if (ret != sizeof(buf)) {
        dev_err(&client->dev, "Failed to read calibration data: %d\n", ret);
        return (ret < 0) ? ret : -EIO;
    }

    calib->AC1 = (s16)((buf[0] << 8) | buf[1]);
    calib->AC2 = (s16)((buf[2] << 8) | buf[3]);
    calib->AC3 = (s16)((buf[4] << 8) | buf[5]);
    calib->AC4 = (u16)((buf[6] << 8) | buf[7]);
    calib->AC5 = (u16)((buf[8] << 8) | buf[9]);
    calib->AC6 = (u16)((buf[10] << 8) | buf[11]);
    calib->B1  = (s16)((buf[12] << 8) | buf[13]);
    calib->B2  = (s16)((buf[14] << 8) | buf[15]);
    calib->MB  = (s16)((buf[16] << 8) | buf[17]);
    calib->MC  = (s16)((buf[18] << 8) | buf[19]);
    calib->MD  = (s16)((buf[20] << 8) | buf[21]);

    dev_info(&client->dev, "Calibration data read successfully\n");
    return 0;
}

static int bmp180_read_raw_temp(struct i2c_client *client, s32 *temp)
{
    int ret;
    u8 buf[2];
    struct bmp180_data *data = i2c_get_clientdata(client);

    mutex_lock(&data->lock);

    ret = i2c_smbus_write_byte_data(client, BMP180_REG_CTRL_MEAS, BMP180_CMD_TEMP);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to start temperature measurement: %d\n", ret);
        goto unlock;
    }

    msleep(BMP180_TEMP_CONVERSION_TIME_MS);

    ret = i2c_smbus_read_i2c_block_data(client, BMP180_REG_OUT_MSB, sizeof(buf), buf);
    if (ret != sizeof(buf)) {
        dev_err(&client->dev, "Failed to read temperature data: %d\n", ret);
        ret = (ret < 0) ? ret : -EIO;
        goto unlock;
    }

    *temp = (buf[0] << 8) | buf[1];
    ret = 0;

unlock:
    mutex_unlock(&data->lock);
    return ret;
}

static int bmp180_read_raw_pressure(struct i2c_client *client, s32 *pressure, int oss)
{
    int ret;
    u8 cmd;
    int delay;
    u8 buf[3];
    struct bmp180_data *data = i2c_get_clientdata(client);

    mutex_lock(&data->lock);

    switch (oss) {
        case 0:
            cmd = BMP180_CMD_PRESSURE_OSS0;
            delay = BMP180_PRESSURE_CONVERSION_TIME_MS(0);
            break;
        case 1:
            cmd = BMP180_CMD_PRESSURE_OSS1;
            delay = BMP180_PRESSURE_CONVERSION_TIME_MS(1);
            break;
        case 2:
            cmd = BMP180_CMD_PRESSURE_OSS2;
            delay = BMP180_PRESSURE_CONVERSION_TIME_MS(2);
            break;
        case 3:
            cmd = BMP180_CMD_PRESSURE_OSS3;
            delay = BMP180_PRESSURE_CONVERSION_TIME_MS(3);
            break;
        default:
            mutex_unlock(&data->lock);
            dev_err(&client->dev, "Invalid oversampling setting: %d\n", oss);
            return -EINVAL;
    }

    ret = i2c_smbus_write_byte_data(client, BMP180_REG_CTRL_MEAS, cmd);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to start pressure measurement: %d\n", ret);
        goto unlock;
    }

    msleep(delay);

    ret = i2c_smbus_read_i2c_block_data(client, BMP180_REG_OUT_MSB, sizeof(buf), buf);
    if (ret != sizeof(buf)) {
        dev_err(&client->dev, "Failed to read pressure data: %d\n", ret);
        ret = (ret < 0) ? ret : -EIO;
        goto unlock;
    }

    *pressure = (buf[0] << 16 | buf[1] << 8 | buf[2]) >> (8 - oss);
    ret = 0;

unlock:
    mutex_unlock(&data->lock);
    return ret;
}

static s32 bmp180_calculate_temp(s32 UT, struct bmp180_calibration *calib)
{
    s32 X1, X2, B5;

    X1 = ((UT - calib->AC6) * calib->AC5) >> 15;
    X2 = (calib->MC << 11) / (X1 + calib->MD);
    B5 = X1 + X2;
    
    return (B5 + 8) >> 4;
}

static s32 bmp180_calculate_pressure(s32 UP, s32 B5, int oss, struct bmp180_calibration *calib)
{
    s32 X1, X2, X3, B3, B6, p;
    u32 B4, B7;

    B6 = B5 - 4000;
    X1 = (calib->B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (calib->AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((s32)calib->AC1 * 4 + X3) << oss) + 2) / 4;

    X1 = (calib->AC3 * B6) >> 13;
    X2 = (calib->B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (calib->AC4 * (u32)(X3 + 32768)) >> 15;

    B7 = ((u32)UP - B3) * (50000 >> oss);
    if (B7 < 0x80000000)
        p = (B7 * 2) / B4;
    else
        p = (B7 / B4) * 2;

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);

    return p;
}

static int bmp180_read_temp_pressure(struct i2c_client *client, s32 *temperature, s32 *pressure)
{
    s32 UT, UP, B5;
    int ret;

    ret = bmp180_read_raw_temp(client, &UT);
    if (ret < 0)
        return ret;

    *temperature = bmp180_calculate_temp(UT, &bmp180_dev->calib);

    ret = bmp180_read_raw_pressure(client, &UP, bmp180_dev->oversampling_setting);
    if (ret < 0)
        return ret;

    s32 X1 = ((UT - bmp180_dev->calib.AC6) * bmp180_dev->calib.AC5) >> 15;
    s32 X2 = (bmp180_dev->calib.MC << 11) / (X1 + bmp180_dev->calib.MD);
    B5 = X1 + X2;

    *pressure = bmp180_calculate_pressure(UP, B5, bmp180_dev->oversampling_setting, &bmp180_dev->calib);

    return 0;
}

static int bmp180_open(struct inode *inode, struct file *filp)
{
    filp->private_data = bmp180_dev;
    return 0;
}

static int bmp180_release(struct inode *inode, struct file *filp)
{
    return 0;
}

static long bmp180_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct bmp180_data *dev = filp->private_data;
    int ret = 0;
    int oss;

    if (_IOC_TYPE(cmd) != BMP180_IOC_MAGIC) 
        return -ENOTTY;
    if (_IOC_NR(cmd) > 3) 
        return -ENOTTY;

    switch (cmd) {
        case BMP180_IOC_SET_OSS:
            if (copy_from_user(&oss, (int __user *)arg, sizeof(oss)))
                return -EFAULT;
            
            if (oss < 0 || oss > 3)
                return -EINVAL;
            
            mutex_lock(&dev->lock);
            dev->oversampling_setting = oss;
            mutex_unlock(&dev->lock);
            
            dev_info(&dev->client->dev, "Oversampling set to %d\n", oss);
            break;

        case BMP180_IOC_GET_OSS:
            mutex_lock(&dev->lock);
            oss = dev->oversampling_setting;
            mutex_unlock(&dev->lock);
            
            if (copy_to_user((int __user *)arg, &oss, sizeof(oss)))
                return -EFAULT;
            break;

        case BMP180_IOC_RESET:
            mutex_lock(&dev->lock);
            dev->oversampling_setting = 0; // Reset to default
            mutex_unlock(&dev->lock);
            
            dev_info(&dev->client->dev, "Device settings reset to defaults\n");
            break;

        default:
            ret = -ENOTTY;
            break;
    }

    return ret;
}

static ssize_t bmp180_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct bmp180_data *dev = filp->private_data;
    s32 temperature, pressure;
    char data_str[50];
    int ret;
    ssize_t len;

    ret = bmp180_read_temp_pressure(dev->client, &temperature, &pressure);
    if (ret < 0)
        return ret;

    len = snprintf(data_str, sizeof(data_str), "Temp: %d.%d C, Pressure: %d Pa (OSS=%d)\n",
                  temperature / 10, abs(temperature % 10), 
                  pressure, dev->oversampling_setting);

    if (len > count)
        len = count;

    if (copy_to_user(buf, data_str, len))
        return -EFAULT;

    return len;
}

static const struct file_operations bmp180_fops = {
    .owner = THIS_MODULE,
    .open = bmp180_open,
    .release = bmp180_release,
    .read = bmp180_read,
    .unlocked_ioctl = bmp180_ioctl,
    .compat_ioctl = bmp180_ioctl,
};

static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device *dev;
    int ret;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C functionality not supported\n");
        return -ENODEV;
    }

    bmp180_dev = devm_kzalloc(&client->dev, sizeof(*bmp180_dev), GFP_KERNEL);
    if (!bmp180_dev)
        return -ENOMEM;

    bmp180_dev->client = client;
    bmp180_dev->oversampling_setting = 0; // Default OSS
    mutex_init(&bmp180_dev->lock);
    i2c_set_clientdata(client, bmp180_dev);

    ret = bmp180_read_calibration(client, &bmp180_dev->calib);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read calibration data\n");
        return ret;
    }

    ret = alloc_chrdev_region(&bmp180_dev->devno, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to allocate char device region\n");
        return ret;
    }

    cdev_init(&bmp180_dev->cdev, &bmp180_fops);
    bmp180_dev->cdev.owner = THIS_MODULE;

    ret = cdev_add(&bmp180_dev->cdev, bmp180_dev->devno, 1);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to add char device\n");
        goto err_cdev;
    }

    bmp180_dev->class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(bmp180_dev->class)) {
        ret = PTR_ERR(bmp180_dev->class);
        dev_err(&client->dev, "Failed to create device class\n");
        goto err_class;
    }

    dev = device_create(bmp180_dev->class, NULL, bmp180_dev->devno, NULL, DEVICE_NAME);
    if (IS_ERR(dev)) {
        ret = PTR_ERR(dev);
        dev_err(&client->dev, "Failed to create device\n");
        goto err_device;
    }

    dev_info(&client->dev, "BMP180 driver initialized successfully\n");
    return 0;

err_device:
    class_destroy(bmp180_dev->class);
err_class:
    cdev_del(&bmp180_dev->cdev);
err_cdev:
    unregister_chrdev_region(bmp180_dev->devno, 1);
    return ret;
}

static void bmp180_remove(struct i2c_client *client)
{
    struct bmp180_data *dev = i2c_get_clientdata(client);

    device_destroy(dev->class, dev->devno);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devno, 1);
    mutex_destroy(&dev->lock);
    
    dev_info(&client->dev, "BMP180 driver removed\n");
}

static const struct i2c_device_id bmp180_id[] = {
    { "bmp180", 0 },
    { "bme180", 0 },
    { "bosch,bmp180", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bmp180_id);

static struct i2c_driver bmp180_driver = {
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
    },
    .probe      = bmp180_probe,
    .remove     = bmp180_remove,
    .id_table   = bmp180_id,
};

module_i2c_driver(bmp180_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("BMP180 I2C Driver with Temperature and Pressure Support");
MODULE_LICENSE("GPL");