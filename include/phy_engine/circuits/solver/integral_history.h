#pragma once
#include <cstddef>
#include <cstdint>

namespace phy_engine::solver
{
    struct integral_history
    {

        constexpr double get(::std::size_t delay) const noexcept { return m_queue[(MaxNumHistoryNum + m_queueHead - delay) & MaxNumHistoryMask]; }

        constexpr void push() noexcept
        {
            m_queueHead = (m_queueHead + 1) & MaxNumHistoryMask;
            m_queue[m_queueHead] = 0.0;
        }

        constexpr void set(::std::size_t delay, double val) noexcept { m_queue[(MaxNumHistoryNum + m_queueHead - delay) & MaxNumHistoryMask] = val; }

        constexpr void setInitial(double val) noexcept
        {
            for(auto c{m_queue}; c != m_queue + MaxNumHistoryNum; ++c) { *c = val; }
        }

        inline static constexpr ::std::size_t MaxNumHistoryShift = 3; /* 2^3 */
        inline static constexpr ::std::size_t MaxNumHistoryNum = (1U << MaxNumHistoryShift);
        inline static constexpr ::std::size_t MaxNumHistoryMask = MaxNumHistoryNum - 1;

        ::std::size_t m_queueHead{};
        double m_queue[MaxNumHistoryNum]{};
    };

}  // namespace phy_engine::solver
