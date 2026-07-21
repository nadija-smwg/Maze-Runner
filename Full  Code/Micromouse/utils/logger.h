/**
 * @file logger.h
 * @brief Serial debug logger with levels.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_INFO  1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_ERROR 3

#define CURRENT_LOG_LEVEL LOG_LEVEL_INFO

#define LOG_DEBUG(...) do { if(CURRENT_LOG_LEVEL <= LOG_LEVEL_DEBUG) { Serial.print("[DEBUG] "); Serial.printf(__VA_ARGS__); Serial.println(); } } while(0)
#define LOG_INFO(...)  do { if(CURRENT_LOG_LEVEL <= LOG_LEVEL_INFO)  { Serial.print("[INFO]  "); Serial.printf(__VA_ARGS__); Serial.println(); } } while(0)
#define LOG_WARN(...)  do { if(CURRENT_LOG_LEVEL <= LOG_LEVEL_WARN)  { Serial.print("[WARN]  "); Serial.printf(__VA_ARGS__); Serial.println(); } } while(0)
#define LOG_ERROR(...) do { if(CURRENT_LOG_LEVEL <= LOG_LEVEL_ERROR) { Serial.print("[ERROR] "); Serial.printf(__VA_ARGS__); Serial.println(); } } while(0)

#endif /* LOGGER_H */
