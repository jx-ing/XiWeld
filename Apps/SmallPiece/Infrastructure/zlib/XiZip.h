/*************************************************************************************************
 * 文件： XiZip.h
 * 说明： zip压缩库，目前仅支持zip压缩
 * 作者： 江文奇
 * 日期： 2025-07-05
 * ToDo： 实现zip解压
 ************************************************************************************************/
#pragma once
#include <string>

#ifdef DLL_API
#undef DLL_API
#endif

#ifdef XIINFRASTRUCTURE_EXPORTS
#define DLL_API 
#else
#define DLL_API 
#endif

#ifdef _DEBUG
#pragma comment (lib, "Apps/SmallPiece/Infrastructure/zlib/debug_x64/zlibd.lib")
#else
#pragma comment (lib, "Apps/SmallPiece/Infrastructure/zlib/release_x64/zlib.lib")
#endif // _DEBUG

namespace xi
{
namespace zip
{
/// @brief 压缩文件夹
/// @param sourcePath 文件夹路径
/// @param zipPath 压缩文件
/// @param pw 压缩密码
/// @return 成功/失败
DLL_API bool ZipDir(std::string sourcePath, const std::string& zipPath, const char* password = nullptr);

/// @brief 压缩文件
/// @param sourcePath 被压缩文件
/// @param zipPath 压缩后文件
/// @param pw 压缩密码
/// @return 成功/失败
DLL_API bool ZipFile(const std::string& sourcePath, const std::string& zipPath, const char* password = nullptr);

}
}

