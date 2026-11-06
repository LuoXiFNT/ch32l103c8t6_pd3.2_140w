/**
 * @file fan_service.h
 * @brief Public interface for the main-board fan control task.
 * @author LuoXiFNT
 * @date 2026-08-23
 * @lastModified 2026-08-23
 */

#ifndef FAN_SERVICE_H

#define FAN_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Calculate board power and update fan PWM duty. */
void Fan_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* FAN_SERVICE_H */
