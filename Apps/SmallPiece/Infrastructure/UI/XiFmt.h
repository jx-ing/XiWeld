/*************************************************************************************************
 * 文件： XiFmt.h
 * 说明： 使特殊类型变量支持fmt格式化字符串，目前提供了CStringW/CStringA支持
 * 作者： 江文奇
 * 日期： 2025-07-05
 * ToDo： 
 ************************************************************************************************/
#pragma once
#include "afx.h"

#pragma warning(push)
#pragma warning(disable: 4996) // 禁用 _DEPRECATE_UNCHECKED 警告
#include "apps/SmallPiece/infrastructure/base/fmt/core.h"
#include "apps/SmallPiece/infrastructure/base/fmt/xchar.h"
#pragma warning(pop) // 恢复警告设置

namespace fmt
{
    template <>
    struct formatter<CStringW, wchar_t>
    {
        template <typename ParseContext>
        auto parse(ParseContext& ctx) -> decltype(ctx.begin())
        {
            auto it = ctx.begin();
            if (it != ctx.end() && *it != '}')
                throw format_error("invalid format");
            return it;
        }

        template <typename FormatContext>
        auto format(const CStringW& str, FormatContext& ctx) const -> decltype(ctx.out())
        {
            return fmt::format_to(ctx.out(), L"{}", static_cast<LPCWSTR>(str));
        }
    };

    template <>
    struct formatter<CStringA, char>
    {
        template <typename ParseContext>
        auto parse(ParseContext& ctx) -> decltype(ctx.begin())
        {
            auto it = ctx.begin();
            if (it != ctx.end() && *it != '}')
                throw format_error("invalid format");
            return it;
        }

        template <typename FormatContext>
        auto format(const CStringA& str, FormatContext& ctx) const -> decltype(ctx.out())
        {
            return fmt::format_to(ctx.out(), "{}", static_cast<LPCSTR>(str));
        }
    };

}
