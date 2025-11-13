#include "stdafx.h"
#include "XiZip.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <direct.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <utility>   
#include <stdlib.h>
#include <io.h>

#include "include/zlib.h"

#include "zip.h"

namespace xi
{
namespace zip
{
// 计算文件 CRC，加密压缩时必须使用
static int getFileCrc(const char* filenameinzip, char* buf, unsigned long size_buf,
	unsigned long* result_crc)
{
	unsigned long calculate_crc = 0;
	int err = ZIP_OK;
	FILE* fin;
	fopen_s(&fin, filenameinzip, "rb");
	size_t size_read = 0;
	if (fin == NULL)
	{
		err = ZIP_ERRNO;
	}

	if (err == ZIP_OK)
		do
		{
			err = ZIP_OK;
			size_read = fread(buf, 1, size_buf, fin);
			if (size_read < size_buf)
				if (feof(fin) == 0)
				{
					printf("error in reading %s\n", filenameinzip);
					err = ZIP_ERRNO;
				}

			if (size_read > 0)
				calculate_crc = crc32_z(calculate_crc, (const Bytef*)buf, size_read);
			/* total_read += size_read; */

		} while ((err == ZIP_OK) && (size_read > 0));

	if (fin)
		fclose(fin);

	*result_crc = calculate_crc;
	printf("file %s crc %lx\n", filenameinzip, calculate_crc);
	return err;
}

// 压缩一个文件
// file--文件名  zf--压缩文件句柄   pw--压缩密码
static int ZipAddFile(int removePathCount, const std::string& file, zipFile& zf, const char* password)
{
	int err = 0;
	char buf[1024];
	size_t len;
	if (password)
	{
		unsigned long crcFile;
		getFileCrc(file.c_str(), buf, sizeof(buf), &crcFile);
		std::string savePath(file);
		savePath = savePath.substr(removePathCount, file.length());
		err = zipOpenNewFileInZip3(zf, savePath.c_str(), NULL, NULL, 0, NULL, 0, NULL, Z_DEFLATED,
			Z_BEST_COMPRESSION, 0, -MAX_WBITS, DEF_MEM_LEVEL, Z_DEFAULT_STRATEGY, password,
			crcFile);
	}
	else
	{
		std::string savePath(file);
		savePath = savePath.substr(removePathCount, file.length());
		err = zipOpenNewFileInZip3(zf, savePath.c_str(), NULL, NULL, 0, NULL, 0, NULL, Z_DEFLATED,
			Z_BEST_COMPRESSION, 0, -MAX_WBITS, DEF_MEM_LEVEL, Z_DEFAULT_STRATEGY, NULL, 0);
	}
	if (err != ZIP_OK)
	{
		return -1;
	}
	FILE* f;
	fopen_s(&f, file.c_str(), "rb");
	if (f == NULL)
	{
		return -2;
	}

	while ((len = fread(buf, 1, sizeof(buf), f)) > 0)
	{
		zipWriteInFileInZip(zf, buf, (unsigned int)len);
	}
	fclose(f);
	zipCloseFileInZip(zf);
	return 0;
}

// removePathCount--移除字符数 sourcePath--文件夹路径  zf--压缩文件句柄 pw--压缩密码
static int ZipAddDir(int removePathCount, const std::string& sourcePath, zipFile& zf, const char* password)
{
	struct _finddata_t fileinfo;
	std::string sourcePathtt = sourcePath + "*.*";
	auto handle = _findfirst(sourcePathtt.c_str(), &fileinfo);
	if ((handle == -1) || (!zf))
	{
		return -1;
	}
	while (!_findnext(handle, &fileinfo))
	{
		// 文件夹
		if (fileinfo.name[0] == '.')
		{
			continue;
		}
		if (fileinfo.attrib & _A_SUBDIR)
		{
			const std::string sourcePathtemp = sourcePath + fileinfo.name + "\\";
			std::string savePath(sourcePathtemp);
			savePath = savePath.substr(removePathCount, sourcePathtemp.length());
			int err = zipOpenNewFileInZip3(zf, savePath.c_str(), NULL, NULL, 0, NULL, 0,
				NULL, Z_DEFLATED, Z_BEST_COMPRESSION, 0, -MAX_WBITS, DEF_MEM_LEVEL, 
				Z_DEFAULT_STRATEGY, NULL, 0);
			if (err != ZIP_OK)
			{
				_findclose(handle);
				return -1;
			}
			zipCloseFileInZip(zf);
			if (ZipAddDir(removePathCount, sourcePathtemp, zf, password) != 0)
			{
				_findclose(handle);
				return -1;
			}
		}
		else
		{
			if (ZipAddFile(removePathCount, sourcePath + fileinfo.name, zf, password) != 0)
			{
				_findclose(handle);
				return -1;
			}
		}
	}
	_findclose(handle);
	return 0;
}

bool ZipDir(std::string sourcePath, const std::string& zipPath, const char* password)
{
	if (sourcePath.back() != '\\' && sourcePath.back() != '/')
		sourcePath = sourcePath + "\\";
	zipFile zf = zipOpen(zipPath.c_str(), APPEND_STATUS_CREATE);
	if (zf == NULL)
	{
		return true;
	}
	int removePathCount = sourcePath.length() - 1;
	std::string savePath(sourcePath);
	savePath = savePath.substr(0, removePathCount);
	removePathCount = savePath.find_last_of("/\\");
	if (removePathCount == std::string::npos)
	{
		removePathCount = 0;
	}
	else
	{
		removePathCount++;
	}
	savePath = savePath.substr(removePathCount, savePath.length());

	zipOpenNewFileInZip3(zf, savePath.c_str(), NULL, NULL, 0, NULL, 0, NULL, Z_DEFLATED,
		Z_BEST_COMPRESSION, 0, -MAX_WBITS, DEF_MEM_LEVEL, Z_DEFAULT_STRATEGY, NULL, 0);
	zipCloseFileInZip(zf);
	int ret1 = ZipAddDir(removePathCount, sourcePath, zf, password);
	int ret2 = zipClose(zf, NULL);
	return (ret1 || ret2);
}

bool ZipFile(const std::string& sourcePath, const std::string& zipPath, const char* password)
{
	zipFile zf = zipOpen(zipPath.c_str(), APPEND_STATUS_CREATE);
	if (zf == NULL)
	{
		return true;
	}

	size_t removePathCount = sourcePath.find_last_of("/\\");
	if (removePathCount == std::string::npos)
	{
		removePathCount = 0;
	}

	std::string savePath(sourcePath);
	savePath = savePath.substr(removePathCount, sourcePath.length());
	zipOpenNewFileInZip3(zf, savePath.c_str(), NULL, NULL, 0, NULL, 0, NULL, Z_DEFLATED,
		Z_BEST_COMPRESSION, 0, -MAX_WBITS, DEF_MEM_LEVEL, Z_DEFAULT_STRATEGY, NULL, 0);
	zipCloseFileInZip(zf);
	int ret1 = ZipAddFile(removePathCount, sourcePath, zf, password);
	int ret2 = zipClose(zf, NULL);
	return (ret1 || ret2);
}
}
}