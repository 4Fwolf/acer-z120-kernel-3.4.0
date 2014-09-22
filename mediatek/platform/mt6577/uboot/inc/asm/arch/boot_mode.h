#ifndef _MT6577_BOOT_MODE_H_
#define _MT6577_BOOT_MODE_H_

/******************************************************************************
 * FORBIDEN MODE
 ******************************************************************************/
typedef enum 
{
  F_FACTORY_MODE = 0x0001,
} FORBIDDEN_MODE;

/******************************************************************************
 * LIMITATION
 ******************************************************************************/
#define SEC_LIMIT_MAGIC  0x4C4C4C4C // LLLL

typedef struct 
{
  unsigned int magic_num;
  FORBIDDEN_MODE forbid_mode;
} SEC_LIMIT;


/* MT6577 boot type definitions */
typedef enum 
{
    NORMAL_BOOT = 0,
    META_BOOT = 1,
    RECOVERY_BOOT = 2,    
    SW_REBOOT = 3,
    FACTORY_BOOT = 4,
    ADVMETA_BOOT = 5,
    ATE_FACTORY_BOOT = 6,
    ALARM_BOOT = 7,
    UNKNOWN_BOOT
} BOOTMODE;

typedef enum {
    BR_POWER_KEY = 0,
    BR_USB,
    BR_RTC,
    BR_WDT,
    BR_WDT_BY_PASS_PWK,
    BR_TOOL_BY_PASS_PWK,
    BR_UNKNOWN
} boot_reason_t;

/*META COM port type*/
 typedef enum
{
    META_UNKNOWN_COM = 0,
    META_UART_COM,
    META_USB_COM
} META_COM_TYPE;

typedef struct {
  u32 addr; /* Download Agend address */
  u32 arg1; /* Download Agend arg 1*/
  u32 arg2; /* Download Agend arg 2*/
} da_info_t;

typedef struct {
  unsigned int maggic_number;
  BOOTMODE boot_mode;
  unsigned int  e_flag;
  unsigned int  log_port;
  unsigned int  log_baudrate;
  unsigned char log_enable;
  unsigned char reserved[3];
  unsigned int dram_rank_num;
  unsigned int dram_rank_size[4];
  unsigned int boot_reason;
  META_COM_TYPE meta_com_type;
  unsigned int meta_com_id;
  unsigned int boot_time;
  da_info_t da_info;
  SEC_LIMIT sec_limit;
} BOOT_ARGUMENT;

/* chip definitions */
typedef enum 
{
    CHIP_E1 = 0xca00,
    CHIP_E2 = 0xcb00,
    CHIP_E3 = 0xcc00, 
    CHIP_6575_E1 = CHIP_E1,
    CHIP_6575_E2 = CHIP_E2,
    CHIP_6575_E3 = CHIP_E3,
    CHIP_6577_E1 = 0xcd00,
} CHIP_VER;

#define BOOT_ARGUMENT_MAGIC 0x504c504c

extern unsigned int BOOT_ARGUMENT_LOCATION;
extern void boot_mode_select(void);
extern BOOTMODE g_boot_mode;
extern CHIP_VER get_chip_eco_ver(void);
extern CHIP_VER get_chip_ver(void);

#endif
