/**************************************************************************//**
 * @file     ps_ram.h
 * @brief    PS RAM configurations device KM
 * @version  V01
 * @date     15 March 2018
 *
 * @note
 *
 ******************************************************************************/

#ifndef PS_RAM_H
#define PS_RAM_H

#ifdef __cplusplus
extern "C" {
#endif

//#include <stdint.h>

/**
 * Configure and Initialize PS RAM
 *
 * @param  none
 * @return none
 *
 * @brief  Necessary to be intialized
 * before systemInit() function in system_KM_app
 */

extern void PsRamSettings(void);

#ifdef __cplusplus
}
#endif

#endif /* PS_RAM_H */
