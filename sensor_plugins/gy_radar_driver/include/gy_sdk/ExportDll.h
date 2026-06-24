#ifndef EXPORTDLL_H
#define EXPORTDLL_H

// 核心修复：
// 如果是在 Windows 编译，保持原样
// 如果是在 Linux 编译，将 DLL_API 定义为空
#if defined(_MSC_VER) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__) || defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
    #ifdef EXPORT_DLL
        #define DLL_API __declspec(dllexport)
    #else
        #define DLL_API __declspec(dllimport)
    #endif
#else
    // Linux / Unix 下定义为空
    #define DLL_API 
#endif

#endif // EXPORTDLL_H