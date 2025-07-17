/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:DateTime.h
简要描述:
******************************************************************/

#ifndef ENGINE_BASE_DATETIME_H_
#define ENGINE_BASE_DATETIME_H_

#include "Base.h"
#include "Base/Types.h"
#include "Base/String.h"
#include <ctime>

namespace Engine
{
    namespace Base
    {
        enum class DSTInfo
        {
            ENABLE = 1,
            DISABLE = 0,
            NOT_AVAILABLE = -1
        };

        class Base_API DateTime
        {
        public:
            /**
             * @brief Date constructor
             * @param year    0-365
             * @param month   1-12
             * @param day     1-31
             */
            DateTime(Int32 year, Int32 month, Int32 day);

            /**
             * @brief Date/time constructor
             * @param year    0-365
             * @param month   1-12
             * @param day     1-31
             * @param hour    0-23
             * @param minute  0-59
             * @param second  0-59
             */
            DateTime(Int32 year, Int32 month, Int32 day, Int32 hour, Int32 minute, Int32 second);

            /**
             * @brief C calendar structure (tm) constructor; http://www.cplusplus.com/reference/ctime/tm/
             * @param timeinfo structure containing a calendar date and time broken down into its components.
             */
            DateTime(struct tm *timeinfo);

            /**
             * @brief Default destructor
             */
            ~DateTime();

            /**
             * @brief Copy constructor
             * @param DateTime object
             */
            DateTime(const DateTime &rhs);

            /**
             * @brief Assigment operator
             * @param DateTime object
             */
            DateTime &operator=(const DateTime &rhs);

            /**
             * @brief Return True if the given date is valid otherwise False
             * @return Bool
             */
            Base::Bool IsValidDate() const;

            /**
             * @brief Set the date part of the DateTime object
             * @param year    0-365
             * @param month   1-12
             * @param day     1-31
             */
            Base::Void SetDate(Int32 year, Int32 month, Int32 day);

            /**
             * @brief Set the time part of the DateTime object
             * @param hour    0-23
             * @param minute  0-59
             * @param second  0-59
             */
            Base::Void SetTime(Int32 hour, Int32 minute, Int32 second);

            /**
             * @brief Timestamp with the number of seconds that have elapsed since January 1, 1970 (midnight UTC/GMT),
             * @return Int64
             */
            Int64 GetTimestamp() const;

            /**
             * @brief True if the year is a leap year otherwise False
             * @return Bool
             */
            static Base::Bool IsLeapYear(Int32 year);

            /**
             * @brief Get a DateTime object with the current local time
             * @return DateTime
             */
            static DateTime GetLocalTime();

            /**
             * @brief Get a DateTime object with the current time in UTC
             * @return DateTime
             */
            static DateTime GetSystemTime();

            /**
             * @brief Get a DateTime (in local time) from a Unix Timestamp (Epoch)
             * @return DateTime
             */
            static DateTime FromEpochToLocalTime(Int64 seconds);

            /**
             * @brief Get a DateTime (in local time) from a Unix Timestamp (Epoch)
             * @return DateTime
             */
            static DateTime FromEpochToSystemTime(Int64 seconds);

            /**
             * @brief Return a String with the date information.
             * @param format date format configuration
             * @return String
             */
            Base::String ToString(const Char *format = nullptr) const;

            /**
             * @brief Different operator
             * @return Bool
             */
            Base::Bool operator!=(const DateTime &rhs) const;

            /**
             * @brief Less operator
             * @return Bool
             */
            Base::Bool operator<(const DateTime &rhs) const;

            /**
             * @brief Less or equal operator
             * @return Bool
             */
            Base::Bool operator<=(const DateTime &rhs) const;

            /**
             * @brief Equal operator
             * @return Bool
             */
            Base::Bool operator==(const DateTime &rhs) const;

            /**
             * @brief Greater operator
             * @return Bool
             */
            Base::Bool operator>(const DateTime &rhs) const;

            /**
             * @brief Greater or equal operator
             * @param DateTime object
             */
            Base::Bool operator>=(const DateTime &rhs) const;

            /**
             * @brief Get C calendar structure (tm)
             * @return tm
             */
            const tm &GetTM() const;

            /**
             * @brief Set time info
             * @param timeinfo
             */
            Void SetTM(const tm &timeinfo);

            /**
             * @brief Set the Daylight Saving Time (DST).
             * @param dst  enum with the different Daylight Saving Time (DST) options.
             */
            Base::Void SetDST(DSTInfo dst);

            /**
             * @brief Get an Enum with the different Daylight Saving Time (DST) options.
             * @return DSTInfo
             */
            DSTInfo GetDST() const;

            /**
             * @brief Set the seconds
             * @param value seconds
             */
            Base::Void SetSecond(Int32 value);

            /**
             * @brief Get the seconds
             * @return Int32
             */
            Base::Int32 GetSecond() const;

            /**
             * @brief Set the minute
             * @param value minutes
             */
            Base::Void SetMinute(Int32 value);

            /**
             * @brief Get the minutes
             * @return Int32
             */
            Base::Int32 GetMinute() const;

            /**
             * @brief Set the hours
             * @param value hours
             */
            Base::Void SetHour(Int32 value);

            /**
             * @brief Get the hours
             * @return Int32
             */
            Base::Int32 GetHour() const;

            /**
             * @brief Set the days
             * @param value days
             */
            Base::Void SetDay(Int32 value);

            /**
             * @brief Get the days
             * @return Int32
             */
            Base::Int32 GetDay() const;

            /**
             * @brief Set the months
             * @param value months
             */
            Base::Void SetMonth(Int32 value);

            /**
             * @brief Get the months
             * @return Int32
             */
            Base::Int32 GetMonth() const;

            /**
             * @brief Set the years
             * @param value years
             */
            Base::Void SetYear(Int32 value);

            /**
             * @brief Get the years
             * @return Int32
             */
            Base::Int32 GetYear() const;

        private:
            struct tm m_TimeInfo;
            time_t m_Timestamp;

            Base::Void MakeTime();
        };
    }
}

#endif // ENGINE_BASE_DATETIME_H_