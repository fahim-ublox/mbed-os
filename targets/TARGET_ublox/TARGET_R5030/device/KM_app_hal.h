/*
 * PackageLicenseDeclared: Apache-2.0
 * Copyright (c) 2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef KM_APP_HAL_H
#define KM_APP_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * Initialize KM hal
 *
 * @param  none
 * @return none
 *
 */
extern void HAL_Init (void);

/**
 * Initialize KM hal
 *
 * @param  none
 * @return none
 *
 */
extern void HAL_SysTick_Config(uint32_t);


#ifdef __cplusplus
}
#endif

#endif /* KM_APP_HAL_H */
