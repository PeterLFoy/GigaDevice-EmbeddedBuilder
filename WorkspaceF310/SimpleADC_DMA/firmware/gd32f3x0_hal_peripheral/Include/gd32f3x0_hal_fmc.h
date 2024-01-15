/*!
    \file    gd32f3x0_hal_fmc.h
    \brief   definitions for the FMC

    \version 2023-08-01, V1.0.0, HAL firmware for GD32F3x0
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifndef GD32F3X0_HAL_FMC_H
#define GD32F3X0_HAL_FMC_H

#include "gd32f3x0.h"
#include <string.h>
#include <stdlib.h>

/* FMC and option byte definition */
#define FMC                     FMC_BASE                    /*!< FMC register base address */
#define OB                      OB_BASE                     /*!< option byte base address */

/* registers definitions */
#define FMC_WS                  REG32(FMC + 0x00000000U)    /*!< FMC wait state register */
#define FMC_KEY                 REG32(FMC + 0x00000004U)    /*!< FMC unlock key register */
#define FMC_OBKEY               REG32(FMC + 0x00000008U)    /*!< FMC option bytes unlock key register */
#define FMC_STAT                REG32(FMC + 0x0000000CU)    /*!< FMC status register */
#define FMC_CTL                 REG32(FMC + 0x00000010U)    /*!< FMC control register */
#define FMC_ADDR                REG32(FMC + 0x00000014U)    /*!< FMC address register */
#define FMC_OBSTAT              REG32(FMC + 0x0000001CU)    /*!< FMC option bytes status register */
#define FMC_WP                  REG32(FMC + 0x00000020U)    /*!< FMC write protection register */
#define FMC_WSEN                REG32(FMC + 0x000000FCU)    /*!< FMC wait state enable register */
#define FMC_PID                 REG32(FMC + 0x00000100U)    /*!< FMC product ID register */

#define OB_SPC                  REG16(OB + 0x00000000U)     /*!< option byte security protection value */
#define OB_USER                 REG16(OB + 0x00000002U)     /*!< option byte user value */
#define OB_DATA0                REG16(OB + 0x00000004U)     /*!< option byte data bit[7:0] value */
#define OB_DATA1                REG16(OB + 0x00000006U)     /*!< option byte data bit[15:8] value */
#define OB_WP0                  REG16(OB + 0x00000008U)     /*!< option byte write protection 0 */
#define OB_WP1                  REG16(OB + 0x0000000AU)     /*!< option byte write protection 1 */

/* bits definitions */
/* FMC_WS */
#define FMC_WS_WSCNT            BITS(0,2)                   /*!< wait state counter */

/* FMC_KEY */
#define FMC_KEY_KEY             BITS(0,31)                  /*!< FMC main flash unlock key bits */

/* FMC_OBKEY */
#define FMC_OBKEY_OBKEY         BITS(0,31)                  /*!< option bytes unlock key bits */

/* FMC_STAT */
#define FMC_STAT_BUSY           BIT(0)                      /*!< flash busy flag bit */
#define FMC_STAT_PGERR          BIT(2)                      /*!< flash program error flag bit */
#define FMC_STAT_WPERR          BIT(4)                      /*!< flash write protection error flag bit */
#define FMC_STAT_ENDF           BIT(5)                      /*!< end of operation flag bit */

/* FMC_CTL */
#define FMC_CTL_PG              BIT(0)                      /*!< main flash program command bit */
#define FMC_CTL_PER             BIT(1)                      /*!< main flash page erase bit */
#define FMC_CTL_MER             BIT(2)                      /*!< main flash mass erase bit */
#define FMC_CTL_OBPG            BIT(4)                      /*!< option bytes program command bit */
#define FMC_CTL_OBER            BIT(5)                      /*!< option bytes erase command bit */
#define FMC_CTL_START           BIT(6)                      /*!< send erase command to FMC bit */
#define FMC_CTL_LK              BIT(7)                      /*!< flash lock bit */
#define FMC_CTL_OBWEN           BIT(9)                      /*!< option bytes erase/program enable bit */
#define FMC_CTL_ERRIE           BIT(10)                     /*!< error interrupt enable bit */
#define FMC_CTL_ENDIE           BIT(12)                     /*!< end of operation interrupt enable bit */
#define FMC_CTL_OBRLD           BIT(13)                     /*!< option bytes reload bit */

/* FMC_ADDR */
#define FMC_ADDR_ADDR           BITS(0,31)                  /*!< flash command address bits */

/* FMC_OBSTAT */
#define FMC_OBSTAT_OBERR        BIT(0)                      /*!< option bytes read error bit */
#define FMC_OBSTAT_PLEVEL_BIT0  BIT(1)                      /*!< protection level bit 0 */
#define FMC_OBSTAT_PLEVEL_BIT1  BIT(2)                      /*!< protection level bit 1 */
#define FMC_OBSTAT_USER         BITS(8,15)                  /*!< option bytes user bits */
#define FMC_OBSTAT_DATA         BITS(16,31)                 /*!< option byte data bits */

/* FMC_WSEN */
#define FMC_WSEN_WSEN           BIT(0)                      /*!< FMC wait state enable bit */
#define FMC_WSEN_BPEN           BIT(1)                      /*!< FMC bit program enable bit */

/* FMC_PID */
#define FMC_PID_PID             BITS(0,31)                  /*!< product ID bits */

/* unlock key */
#define UNLOCK_KEY0             ((uint32_t)0x45670123U)     /*!< unlock key 0 */
#define UNLOCK_KEY1             ((uint32_t)0xCDEF89ABU)     /*!< unlock key 1 */

/* wait state counter value */
#define WS_WSCNT_0              ((uint8_t)0x00U)            /*!< 0 wait state added */
#define WS_WSCNT_1              ((uint8_t)0x01U)            /*!< 1 wait state added */
#define WS_WSCNT_2              ((uint8_t)0x02U)            /*!< 2 wait state added */

/* read protect configure */
#define FMC_NSPC                ((uint8_t)0xA5U)            /*!< no security protection */
#define FMC_LSPC                ((uint8_t)0xBBU)            /*!< low security protection, any value except 0xA5 or 0xCC */
#define FMC_HSPC                ((uint8_t)0xCCU)            /*!< high security protection */

/* option byte write protection */
#define OB_LWP                  ((uint32_t)0x000000FFU)     /*!< write protection low bits */
#define OB_HWP                  ((uint32_t)0x0000FF00U)     /*!< write protection high bits */

#define OB_FWDGT_HW             ((uint8_t)(~BIT(0)))        /*!< hardware free watchdog timer */
#define OB_DEEPSLEEP_RST        ((uint8_t)(~BIT(1)))        /*!< generate a reset instead of entering deepsleep mode */
#define OB_STDBY_RST            ((uint8_t)(~BIT(2)))        /*!< generate a reset instead of entering standby mode */
#define OB_BOOT1_SET_1          ((uint8_t)(~BIT(4)))        /*!< BOOT1 bit is 1 */
#define OB_VDDA_DISABLE         ((uint8_t)(~BIT(5)))        /*!< disable VDDA monitor */
#define OB_SRAM_PARITY_ENABLE   ((uint8_t)(~BIT(6)))        /*!< enable SRAM parity check */

/* option byte security protection level in FMC_OBSTAT register */
#define OB_OBSTAT_PLEVEL_NO     ((uint32_t)0x00000000U)     /*!< no security protection */
#define OB_OBSTAT_PLEVEL_LOW    ((uint32_t)0x00000002U)     /*!< low security protection */
#define OB_OBSTAT_PLEVEL_HIGH   ((uint32_t)0x00000006U)     /*!< high security protection */

#define OB_USER_DEFAULT         ((uint8_t)0xDFU)            /*!< OB_USER default value */

/* option byte parameter address */
#define OB_SPC_ADDR             (uint32_t)(OB + 0x00000000U)/*!< option byte spc address */
#define OB_USER_ADDR            (uint32_t)(OB + 0x00000002U)/*!< option byte user address */
#define OB_DATA_ADDR0           (uint32_t)(OB + 0x00000004U)/*!< option byte data address 0 */
#define OB_DATA_ADDR1           (uint32_t)(OB + 0x00000006U)/*!< option byte data address 1 */
#define OB_WP_ADDR0             (uint32_t)(OB + 0x00000008U)/*!< option byte wp address 0 */
#define OB_WP_ADDR1             (uint32_t)(OB + 0x0000000AU)/*!< option byte wp address 1 */

/* FMC flags */
#define FMC_FLAG_BUSY           FMC_STAT_BUSY               /*!< FMC busy flag */
#define FMC_FLAG_PGERR          FMC_STAT_PGERR              /*!< FMC programming error flag */
#define FMC_FLAG_WPERR          FMC_STAT_WPERR              /*!< FMC write protection error flag */
#define FMC_FLAG_END            FMC_STAT_ENDF               /*!< FMC end of programming flag */

/* FMC interrupt enable */
#define FMC_INTEN_END           FMC_CTL_ENDIE               /*!< enable FMC end of operation interrupt */
#define FMC_INTEN_ERR           FMC_CTL_ERRIE               /*!< enable FMC error interrupt */

/* FMC time out */
#define FMC_TIMEOUT_COUNT       ((uint32_t)0x000F0000U)     /*!< count to judge of FMC timeout */

/* option byte parameter */
typedef struct {
    uint8_t spc;                                            /*!< option byte parameter spc */
    uint8_t user;                                           /*!< option byte parameter user */
    uint8_t data0;                                          /*!< option byte parameter data0 */
    uint8_t data1;                                          /*!< option byte parameter data1 */
    uint8_t wp0;                                            /*!< option byte parameter wp0 */
    uint8_t wp1;                                            /*!< option byte parameter wp1 */
} ob_parm_struct;

/* constants definitions */
/* fmc state */
typedef enum {
    FMC_READY,                                              /*!< the operation has been completed */
    FMC_BUSY,                                               /*!< the operation is in progress */
    FMC_PGERR,                                              /*!< program error */
    FMC_WPERR,                                              /*!< erase/program protection error */
    FMC_TOERR,                                              /*!< timeout error */
    FMC_OB_HSPC                                             /*!< option byte security protection code high */
} fmc_state_enum;

/* PARA: sector_addr_range */
/* @STRUCT-00: sector address range */
/* @HAL_SECTOR_ADDR_RANGE_STRUCT */
typedef struct{
    uint32_t sector_start_addr;         /*!< sector actual start address */ 
    uint32_t sector_end_addr;           /*!< sector actual end address */ 
} hal_sector_addr_range_struct;


/* PARA: fmc_irq */
/* @STRUCT-02: fmc irq struct */
/* @HAL_FMC_IRQ_STRUCT */
typedef struct {
    hal_irq_handle_cb error_handle;     /*!< flash error interrupt */ 
    hal_irq_handle_cb finish_handle;    /*!< end of operation interrupt */  
} hal_fmc_irq_struct;

/* PARA: ob_parm_config */
/* @STRUCT-03: option byte parameter */
/* @HAL_OB_PARM_CONFIG_STRUCT */
typedef struct  
{
    uint8_t ob_type;                    /*!< ob_type: OB_TYPE_WRP,OB_TYPE_SPC,OB_TYPE_USER and OB_TYPE_DATA.
                                        one or more parameters can be selected which are shown as above */

    uint8_t wp_state;                  /*!< wp_state: OB_WP_DISABLE or OB_WP_ENABLE */

    uint32_t wp_addr;                   /*!< wp_addr: specifies the target start address */

    uint32_t wp_size;                   /*!< wp_size: specifies the data size to be write protected */
  
    uint8_t spc_level;                  /*!< spc_level: OB_SPC_0, OB_SPC_1 or  OB_SPC_2
                                        only one parameter can be selected which is shown as above */
  
    uint8_t user;                       /*!< user: the value is used to configure OB_USER*/
  
    uint16_t data_value;                /*!< data_value:  the low byte of data_value is used to configure OB_DATA0
                                        the high byte of data_value is used to configure OB_DATA1 */  
} hal_ob_parm_config_struct;

/* @@STRUCT_MEMBER00: ob_type */
/* @@DEFINE-00: option byte type selection */
#define OB_TYPE_WP        0x01U         /*!< WP option byte configuration */
#define OB_TYPE_SPC       0x02U         /*!< SPC option byte configuration */
#define OB_TYPE_USER      0x04U         /*!< USER option byte configuration */
#define OB_TYPE_DATA      0x08U         /*!< DATA option byte configuration */

/* @@STRUCT_MEMBER01: wp_state */
/* @@DEFINE-01: enanle or disable the write protection */
#define OB_WP_DISABLE     0x00U         /*!< disable the write protection of the targeted pages */
#define OB_WP_ENABLE      0x01U         /*!< enable the write protection of the targeted pages */

/* @@STRUCT_MEMBER02: wp_addr */
/* =NULL */

/* @@STRUCT_MEMBER03: wp_size */
/* =NULL */

/* @@STRUCT_MEMBER04: spc_level */
/* @@DEFINE-04: security protection level seclection */
#define OB_SPC_0          0xA5U         /*!< no security protection */
#define OB_SPC_1          0xBBU         /*!< low security protection */
#define OB_SPC_2          0xCCU         /*!< high security protection */

/* @@STRUCT_MEMBER05: user */
/* =NULL */

/* @@STRUCT_MEMBER06: data_value */
/* =NULL */

#define FMC_BASE_ADDRESS 0x8000000U     /*!< main flash start address*/
#define FMC_END_ADDRESS  0x8010000U     /*!< main flash max end address*/
#define FMC_PAGE_SIZE    0x400U         /*!< main flash page size */
#define FMC_SECTOR_SIZE  0x1000U        /*!< main flash sector size */

/* function declarations */
/* main flash operation */
/* unlock the main FMC operation */
int32_t hal_fmc_unlock(void);
/* lock the main FMC operation */
int32_t hal_fmc_lock(void);
/* enable fmc wait state */
void hal_fmc_wait_state_enable(void);
/* disable fmc wait state */
void hal_fmc_wait_state_disable(void);
/* check whether FMC is ready or not */
fmc_state_enum hal_fmc_ready_wait(uint32_t timeout);
/* set the wait state counter value */
void hal_fmc_wscnt_set(uint8_t wscnt);
/* read flash target region */
void hal_fmc_region_read(uint32_t start_addr, uint8_t *data, uint32_t size);
/* write flash target address in word */
fmc_state_enum hal_fmc_word_program(uint32_t addr, uint32_t data);
/* write flash target address in halfword */
fmc_state_enum hal_fmc_halfword_program(uint32_t addr, uint16_t data);
/* write flash target region with amounts of data */
int32_t hal_fmc_region_write(uint32_t start_addr, uint8_t *data, uint32_t data_size);
/* erase the page which start address locating in */
fmc_state_enum hal_fmc_page_erase(uint32_t start_addr);
/* erase the whole flash */
fmc_state_enum hal_fmc_mass_erase(void);
/* erase flash target region */
int32_t hal_fmc_region_erase(uint32_t start_addr, uint32_t size);

/* interrupt configuration */
/* fmc interrupt handler content function,which is merely used in fmc_handler */
void hal_fmc_irq(void);
/* set user-defined interrupt callback function, which will be registered and called when corresponding interrupt be triggered */
void hal_fmc_irq_handle_set(hal_fmc_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, which will be registered and called when corresponding interrupt be triggered */
void hal_fmc_irq_handle_all_reset(hal_fmc_irq_struct *p_irq);

/* option byte operation */
/* unlock option byte */
void hal_ob_unlock(void);
/* lock option byte */
void hal_ob_lock(void);
/* reset option byte */
void hal_ob_reset(void);
/* erase option byte */
fmc_state_enum hal_ob_erase(void);
/* configure option byte security protection */
fmc_state_enum hal_ob_security_protection_config(uint8_t ob_spc);
/* write option byte user */
fmc_state_enum hal_ob_user_write(uint8_t ob_user);
/* program the FMC data option byte */
fmc_state_enum hal_ob_data_program(uint32_t address, uint8_t ob_data);
/* enable the targeted address region written protection */
hal_sector_addr_range_struct hal_ob_wp_enable(uint32_t start_addr, uint32_t data_size);
/* disable the targeted address region written protection */
hal_sector_addr_range_struct hal_ob_wp_disable(uint32_t start_addr, uint32_t data_size);
/* get option byte parameters, which are stored in register FMC_OBSTAT and FMC_WP */
int32_t hal_ob_parm_get(ob_parm_struct *p_parm);
/* enable option byte write protection(OB_WP) depending on current option byte */
fmc_state_enum hal_ob_write_protection_enable(uint16_t ob_wp);
/* configure option byte parameters thoroughly */
int32_t hal_ob_parm_config(hal_ob_parm_config_struct *ob_parm);
/* modify the target option byte depending on the original value */
void _ob_value_modify(uint32_t address, uint16_t value, ob_parm_struct *ob_parm);
/* erase page */
fmc_state_enum _fmc_page_erase(uint32_t page_address);
/* get the FMC state */
fmc_state_enum _fmc_state_get(void);
/* get FMC flag */
FlagStatus _fmc_flag_get(uint32_t flag);
/* get FMC interrupt flag */
FlagStatus _fmc_interrupt_flag_get(uint32_t flag);
#endif /* GD32F3X0_HAL_FMC_H */
