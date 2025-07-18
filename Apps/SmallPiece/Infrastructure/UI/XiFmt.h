/*************************************************************************************************
 * �ļ��� XiFmt.h
 * ˵���� ʹ�������ͱ���֧��fmt��ʽ���ַ�����Ŀǰ�ṩ��CStringW/CStringA֧��
 * ���ߣ� ������
 * ���ڣ� 2025-07-05
 * ToDo�� 
 ************************************************************************************************/
#pragma once
#include "afx.h"

#pragma warning(push)
#pragma warning(disable: 4996) // ���� _DEPRECATE_UNCHECKED ����
#include "apps/SmallPiece/infrastructure/base/fmt/core.h"
#include "apps/SmallPiece/infrastructure/base/fmt/xchar.h"
#pragma warning(pop) // �ָ���������

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
