/*  mlx90614 Driver for MTK
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/math64.h>

#include "temperature.h"

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("mlx90614 Driver for MTK");
MODULE_AUTHOR("tianyx < zzztyx55@sina.com >");

//#define EN_DEBUG

#define DBG_TAG                "mlx90614_temperature"

#ifdef EN_DEBUG
    #define dbg_mesg(format, args...)										\
		    do{																\
			    printk(KERN_DEBUG DBG_TAG": %s() _%d_: " format	            \
					    , __FUNCTION__										\
					    , __LINE__											\
					    , ## args);											\
		    }while(0)
#else
    #define dbg_mesg(format, args...)     ((void)0)
#endif

#define alert_mesg(format, args...)										\
		do{																\
			printk(KERN_ERR DBG_TAG": %s() _%d_: " format	            \
					, __FUNCTION__										\
					, __LINE__											\
					, ## args);											\
		}while(0)



#define MLX90614_DEV_NAME        "mlx90614_dev"

#define ACK           0
#define NACK          1


#define MLX90614_I2C_ADDR    (0x5a)  //0x00 //Slave address 单个MLX90614时地址为0x00,多个时地址默认为0x5a

/* Device registers */
#define MLX90614_REG_OBJ_TEMP		0x06
#define RAM_ACCESS                  0x00 //RAM access command
#define EEPROM_ACCESS               0x20 //EEPROM access command
#define RAM_TOBJ1                   0x07 //To1 address in the eeprom


//#define MLX90614_SDA_PIN         (GPIO24 | 0x80000000)
#define MLX90614_SDA_PIN         (24 | 0x80000000)

#define SMBUS_SDA_PIN    mt_get_gpio_in(MLX90614_SDA_PIN) //读取引脚电平

#define SDA_L     pinctrl_select_state(pinctrl, mlx90614_sda_out_l)  
#define SDA_H     pinctrl_select_state(pinctrl, mlx90614_sda_out_h)  
#define SCL_H     pinctrl_select_state(pinctrl, mlx90614_scl_out_h)
#define SCL_L     pinctrl_select_state(pinctrl, mlx90614_scl_out_l)


static struct pinctrl *pinctrl;
static struct pinctrl_state *mlx90614_scl_out_l, *mlx90614_scl_out_h, *mlx90614_sda_out_l, 
		*mlx90614_sda_out_h, *mlx90614_sda_input, *mlx90614_pwr_en, *mlx90614_pwr_off;


extern int mt_get_gpio_in(unsigned long pin);

static void SMBus_StartBit(void);
static void SMBus_StopBit(void);
static void SMBus_SendBit(u8);
static u8 SMBus_SendByte(u8);
static u8 SMBus_ReceiveBit(void);
static u8 SMBus_ReceiveByte(u8);
static void SMBus_Delay(u16);
static u8 PEC_Calculation(u8*);
static u16 SMBus_ReadMemory(u8, u8);

/*******************************************************************************
* Function Name  : SMBus_StartBit
* Description    : Generate START condition on SMBus
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void SMBus_StartBit(void)
{
    SDA_H;               // Set SDA line 
    SMBus_Delay(1);      // Wait a few microseconds 
    SCL_H;               // Set SCK line  
    SMBus_Delay(5);      // Generate bus free time between Stop
    SDA_L;               // Clear SDA line
    SMBus_Delay(10);     // Hold time after (Repeated) Start
                         // Condition. After this period, the first clock is generated.
                         //(Thd:sta=4.0us min)
    SCL_L;               // Clear SCK line
    SMBus_Delay(2);      // Wait a few microseconds
}


/*******************************************************************************
* Function Name  : SMBus_StopBit
* Description    : Generate STOP condition on SMBus
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void SMBus_StopBit(void)
{
    SCL_L;                // Clear SCK line
    SMBus_Delay(5);       // Wait a few microseconds
    SDA_L;                // Clear SDA line
    SMBus_Delay(5);       // Wait a few microseconds
    SCL_H;                // Set SCK line
    SMBus_Delay(10);      // Stop condition setup time(Tsu:sto=4.0us min)
    SDA_H;                // Set SDA line
}

/*******************************************************************************
* Function Name  : SMBus_SendByte
* Description    : Send a byte on SMBus
* Input          : Tx_buffer
* Output         : None
* Return         : None
*******************************************************************************/
static u8 SMBus_SendByte(u8 Tx_buffer)
{
    u8        Bit_counter;
    u8        Ack_bit;
    u8        bit_out;


    for(Bit_counter=8; Bit_counter; Bit_counter--)
    {
        if (Tx_buffer&0x80)
        {
            bit_out=1;       // If the current bit of Tx_buffer is 1 set bit_out
        }
        else
        {
            bit_out=0;      // else clear bit_out
        }
        SMBus_SendBit(bit_out);           // Send the current bit on SDA
        Tx_buffer<<=1;                    // Get next bit for checking
    }
    Ack_bit=SMBus_ReceiveBit();           // Get acknowledgment bit
    return        Ack_bit;
}

/*******************************************************************************
* Function Name  : SMBus_SendBit
* Description    : Send a bit on SMBus
* Input          : bit_out
* Output         : None
* Return         : None
*******************************************************************************/
static void SMBus_SendBit(u8 bit_out)
{
    if(bit_out==0)
    {
      SDA_L;   
    }
    else
    {
    SDA_H;
    }
    SMBus_Delay(2);                            // Tsu:dat = 250ns minimum
    SCL_H;                                     // Set SCK line
    SMBus_Delay(10);                           // High Level of Clock Pulse
    SCL_L;                                     // Clear SCK line
    SMBus_Delay(10);                           // Low Level of Clock Pulse
//        SMBUS_SDA_H();                       // Master release SDA line ,
    return;
}
/*******************************************************************************
* Function Name  : SMBus_ReceiveBit
* Description    : Receive a bit on SMBus
* Input          : None
* Output         : None
* Return         : Ack_bit
*******************************************************************************/
static u8 SMBus_ReceiveBit(void)
{
    u8 Ack_bit;


    SDA_H;             //引脚靠外部电阻上拉，当作输入
    SCL_H;             // Set SCL line
    SMBus_Delay(2);    // High Level of Clock Pulse
    pinctrl_select_state(pinctrl, mlx90614_sda_input); // set sda as input mode    
    if (SMBUS_SDA_PIN)
    {
        Ack_bit=1;
    }
    else
    {
        Ack_bit=0;
    }
    SCL_L;                    // Clear SCL line
    SMBus_Delay(4);           // Low Level of Clock Pulse
    return   Ack_bit;
}

/*******************************************************************************
* Function Name  : SMBus_ReceiveByte
* Description    : Receive a byte on SMBus
* Input          : ack_nack
* Output         : None
* Return         : RX_buffer
*******************************************************************************/
static u8 SMBus_ReceiveByte(u8 ack_nack)
{
    u8        RX_buffer = 0;
    u8        Bit_Counter = 0;
    for(Bit_Counter=8; Bit_Counter; Bit_Counter--)
    {
        if(SMBus_ReceiveBit())         // Get a bit from the SDA line
        {
            RX_buffer <<= 1;           // If the bit is HIGH save 1  in RX_buffer
            RX_buffer |=0x01;
        }
        else
        {
            RX_buffer <<= 1;           // If the bit is LOW save 0 in RX_buffer
            RX_buffer &=0xfe;
        }
    }
    SMBus_SendBit(ack_nack);           // Sends acknowledgment bit
    return RX_buffer;
}

/*******************************************************************************
* Function Name  : SMBus_Delay
* Description    : 延时  一次循环约1us
* Input          : time
* Output         : None
* Return         : None
*******************************************************************************/
static void SMBus_Delay(u16 time)
{
    u16 i, j;
    for (i=0; i<4; i++)
    {
        for (j=0; j<time; j++);
    }
}


/*******************************************************************************
 * Function Name  : SMBus_ReadMemory
 * Description    : READ DATA FROM RAM/EEPROM
 * Input          : slaveAddress, command
 * Output         : None
 * Return         : Data
*******************************************************************************/
static u16 SMBus_ReadMemory(u8 slaveAddress, u8 command)
{
    u16 data;               // Data storage (DataH:DataL)
    u8 Pec;                 // PEC byte storage
    u8 DataL=0;             // Low data byte storage
    u8 DataH=0;             // High data byte storage
    u8 arr[6];              // Buffer for the sent bytes
    u8 PecReg;              // Calculated PEC byte storage
    u8 ErrorCounter;        // Defines the number of the attempts for communication with MLX90614


    ErrorCounter=0x00;                                // Initialising of ErrorCounter
        slaveAddress <<= 1;        //2-7位表示从机地址

    do
    {
repeat:
        SMBus_StopBit();                //If slave send NACK stop comunication
        --ErrorCounter;                 //Pre-decrement ErrorCounter
        if(!ErrorCounter)               //ErrorCounter=0?
        {
            break;                      //Yes,go out from do-while{}
        }

        SMBus_StartBit();               //Start condition
        if(SMBus_SendByte(slaveAddress))//Send SlaveAddress 最低位Wr=0表示接下来写命令
        {
            goto  repeat;               //Repeat comunication again
        }
        if(SMBus_SendByte(command))     //Send command
        {
            goto    repeat;             //Repeat comunication again
        }

        SMBus_StartBit();                //Repeated Start condition
        if(SMBus_SendByte(slaveAddress+1))  //Send SlaveAddress 最低位Rd=1表示接下来读数据
        {
            goto        repeat;           //Repeat comunication again
        }

        DataL = SMBus_ReceiveByte(ACK);   //Read low data,master must send ACK
        DataH = SMBus_ReceiveByte(ACK);   //Read high data,master must send ACK
        Pec = SMBus_ReceiveByte(NACK);    //Read PEC byte, master must send NACK
        SMBus_StopBit();                  //Stop condition

        arr[5] = slaveAddress;        
        arr[4] = command;
        arr[3] = slaveAddress+1;         //Load array arr
        arr[2] = DataL;                 
        arr[1] = DataH;                
        arr[0] = 0;                   
        PecReg=PEC_Calculation(arr);     //Calculate CRC
    }
    while(PecReg != Pec);                //If received and calculated CRC are equal go out from do-while{}
        data = (DataH<<8) | DataL;       //data=DataH:DataL
    return data;
}



/*******************************************************************************
* Function Name  : PEC_calculation
* Description    : Calculates the PEC of received bytes
* Input          : pec[]
* Output         : None
* Return         : pec[0]-this byte contains calculated crc value
*******************************************************************************/
static u8 PEC_Calculation(u8 pec[])
{
    u8         crc[6];
    u8        BitPosition=47;
    u8        shift;
    u8        i;
    u8        j;
    u8        temp;


    do
    {
        /*Load pattern value 0x000000000107*/
        crc[5]=0;
        crc[4]=0;
        crc[3]=0;
        crc[2]=0;
        crc[1]=0x01;
        crc[0]=0x07;
        /*Set maximum bit position at 47 ( six bytes byte5...byte0,MSbit=47)*/
        BitPosition=47;
        /*Set shift position at 0*/
        shift=0;
        /*Find first "1" in the transmited message beginning from the MSByte byte5*/
        i=5;
        j=0;
        while((pec[i]&(0x80>>j))==0 && i>0)
        {
            BitPosition--;
            if(j<7)
            {
                j++;
            }
            else
            {
                j=0x00;
                i--;
            }
        }/*End of while */


        /*Get shift value for pattern value*/
        shift=BitPosition-8;
        /*Shift pattern value */
        while(shift)
        {
            for(i=5; i<0xFF; i--)
            {
                if((crc[i-1]&0x80) && (i>0))
                {
                    temp=1;
                }
                else
                {
                    temp=0;
                }
                crc[i]<<=1;
                crc[i]+=temp;
            }/*End of for*/
            shift--;
        }/*End of while*/
        /*Exclusive OR between pec and crc*/
        for(i=0; i<=5; i++)
        {
            pec[i] ^=crc[i];
        }/*End of for*/
    }
    while(BitPosition>8); /*End of do-while*/

    return pec[0];
}

/*******************************************************************************
 * Function Name  : SMBus_ReadTemp
 * Description    : Calculate and return the temperature
 * Input          : None
 * Output         : None
 * Return         : SMBus_ReadMemory(0x00, 0x07)*0.02-273.15
*******************************************************************************/
/* driver do not support float
float SMBus_ReadTemp(void)
{   
    return SMBus_ReadMemory(SA, RAM_ACCESS|RAM_TOBJ1)*0.02-273.15;
}
*/


static int mlx90614_get_temperature(int* value, int * status)
{
    u16 temp;

	//float temp = SMBus_ReadTemp();    
    temp = SMBus_ReadMemory(MLX90614_I2C_ADDR, RAM_ACCESS|RAM_TOBJ1);

    *value = temp;
    *status = 0;
	return 0;
}


static int mlx90614_enable_nodata(int en)
{
    dbg_mesg("++, en=%d\n", en);
    
    if(en)
        pinctrl_select_state(pinctrl, mlx90614_pwr_en);
    else
        pinctrl_select_state(pinctrl, mlx90614_pwr_off);    
    
    return 0;
}

static int mlx90614_set_delay(u64 ns)
{
    dbg_mesg("++\n");
 	return 0;
}

static int mlx90614_open_report_data(int open)
{
    dbg_mesg("++,open:%d\n", open);
 	return 0;
}



static int mlx90614_pinctl_init(struct platform_device *pdev)
{
    int ret;
    
    pinctrl = devm_pinctrl_get(&pdev->dev);
    if (IS_ERR(pinctrl)) {
            ret = PTR_ERR(pinctrl);
            dev_err(&pdev->dev, "Cannot find mlx90614 pinctrl!\n");
            return ret;
    }   
    
    mlx90614_scl_out_l = pinctrl_lookup_state(pinctrl, "mlx90614_scl_out_l");
    if (IS_ERR(mlx90614_scl_out_l)) {
            ret = PTR_ERR(mlx90614_scl_out_l);
            dev_err(&pdev->dev, "Cannot find mlx90614_scl_out_l %d!\n", ret);
            return ret;
    }   
    mlx90614_scl_out_h = pinctrl_lookup_state(pinctrl, "mlx90614_scl_out_h");
    if (IS_ERR(mlx90614_scl_out_h)) {
            ret = PTR_ERR(mlx90614_scl_out_h);
            dev_err(&pdev->dev, "Cannot find mlx90614_scl_out_h %d!\n", ret);
            return ret;
    }   
    mlx90614_sda_out_l = pinctrl_lookup_state(pinctrl, "mlx90614_sda_out_l");
    if (IS_ERR(mlx90614_sda_out_l)) {
            ret = PTR_ERR(mlx90614_sda_out_l);
            dev_err(&pdev->dev, "Cannot find mlx90614_sda_out_l %d!\n", ret);
            return ret;
    }    
    mlx90614_sda_out_h = pinctrl_lookup_state(pinctrl, "mlx90614_sda_out_h");
    if (IS_ERR(mlx90614_sda_out_h)) {
            ret = PTR_ERR(mlx90614_sda_out_h);
            dev_err(&pdev->dev, "Cannot find mlx90614_sda_out_h %d!\n", ret);
            return ret;
    }  

    mlx90614_sda_input = pinctrl_lookup_state(pinctrl, "mlx90614_sda_input");
    if (IS_ERR(mlx90614_sda_input)) {
            ret = PTR_ERR(mlx90614_sda_input);
            dev_err(&pdev->dev, "Cannot find mlx90614_sda_input %d!\n", ret);
            return ret;
    }  

    mlx90614_pwr_en = pinctrl_lookup_state(pinctrl, "mlx90614_pwr_en");
    if (IS_ERR(mlx90614_pwr_en)) {
            ret = PTR_ERR(mlx90614_pwr_en);
            dev_err(&pdev->dev, "Cannot find mlx90614_pwr_en %d!\n", ret);
            return ret;
    }   

    mlx90614_pwr_off = pinctrl_lookup_state(pinctrl, "mlx90614_pwr_off");
    if (IS_ERR(mlx90614_pwr_off)) {
            ret = PTR_ERR(mlx90614_pwr_off);
            dev_err(&pdev->dev, "Cannot find mlx90614_pwr_off %d!\n", ret);
            return ret;
    }   


    pinctrl_select_state(pinctrl, mlx90614_pwr_off);
    pinctrl_select_state(pinctrl, mlx90614_sda_out_h);
    pinctrl_select_state(pinctrl, mlx90614_scl_out_h);
    
    return 0;
}

static int mlx90614_probe(struct platform_device *pdev)
{
    struct temp_control_path ctl = { 0 };
    struct temp_data_path data = { 0 };
	int rc;
    
    dbg_mesg("mlx90614 %s entry\n", __func__);

    rc = mlx90614_pinctl_init(pdev);
    if(rc)
    {
        return rc;
    }

#if 0
	pinctrl_select_state(pinctrl, mlx90614_pwr_en);
	val = SMBus_ReadMemory(MLX90614_I2C_ADDR, RAM_ACCESS|0x1c);
    dbg_mesg("mlx90614 CHIP ID:0x%x\n", val);
	val = SMBus_ReadMemory(MLX90614_I2C_ADDR, RAM_ACCESS|0x1d);
    dbg_mesg("mlx90614 CHIP ID:0x%x\n", val);
	val = SMBus_ReadMemory(MLX90614_I2C_ADDR, RAM_ACCESS|0x1e);
    dbg_mesg("mlx90614 CHIP ID:0x%x\n", val);
	val = SMBus_ReadMemory(MLX90614_I2C_ADDR, RAM_ACCESS|0x1f);
    dbg_mesg("mlx90614 CHIP ID:0x%x\n", val);
    pinctrl_select_state(pinctrl, mlx90614_pwr_off);
#endif    
    
    ctl.open_report_data = mlx90614_open_report_data;
    ctl.enable_nodata = mlx90614_enable_nodata;
    ctl.set_delay = mlx90614_set_delay;
    ctl.is_report_input_direct = false;
    ctl.is_support_batch = false;
    ctl.is_use_common_factory = false;
    rc = temp_register_control_path(&ctl);
    if (rc) {
		goto err_out;
    }

    data.get_data = mlx90614_get_temperature;
    data.vender_div = 50;
    rc = temp_register_data_path(&data);
    if (rc) {
		goto err_out;
    }

    dbg_mesg("mlx90614 %s success\n", __func__);

	return 0;
err_out:
    return rc;
}

static int mlx90614_remove(struct platform_device *pdev)
{
	return 0;
}

#if !defined(CONFIG_HAS_EARLYSUSPEND)
static int  mlx90614_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int mlx90614_resume(struct platform_device *pdev)
{
    return 0;
}
#endif

#ifdef CONFIG_OF
static const struct of_device_id mlx90614_of_match[] = {
	{.compatible = "mediatek,mlx90614"},
	{},
};
#endif


static struct platform_driver  mlx90614_driver = {
	.driver = {
		   .name = MLX90614_DEV_NAME,
		   .owner  = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mlx90614_of_match,
#endif
    },
	.probe = mlx90614_probe,
	.remove = mlx90614_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = mlx90614_suspend,
	.resume = mlx90614_resume,
#endif
};

static int mlx90614_uninit(void) 
{
    dbg_mesg("++\n");
    platform_driver_unregister(&mlx90614_driver);
    return 0;
}

static int  mlx90614_local_init(void) 
{
    dbg_mesg("++\n");
    return platform_driver_register(&mlx90614_driver);
}


static struct temp_init_info mlx90614_init_info = {
    .name = "mlx90614",
    .init = mlx90614_local_init,
    .uninit = mlx90614_uninit,
};

static int __init mlx90614_init(void) 
{
    dbg_mesg("++\n");

    temp_driver_add(&mlx90614_init_info);
    return 0;
}

static void __exit mlx90614_exit(void) 
{
    dbg_mesg("++\n");
}

module_init(mlx90614_init);
module_exit(mlx90614_exit);


