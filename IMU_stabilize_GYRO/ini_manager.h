#pragma once
#include <Windows.h>
#include <string>
#include <vector>

class INIManager
{
public:
    INIManager(const std::wstring& fileName);

    int ReadInteger(const std::wstring& section, const std::wstring& key, int defaultValue);
    float ReadFloat(const std::wstring& section, const std::wstring& key, float defaultValue);
    bool ReadBoolean(const std::wstring& section, const std::wstring& key, bool defaultValue);
    double ReadDouble(const std::wstring& section, const std::wstring& key, double defaultValue);
    bool ReadString(const std::wstring& section, const std::wstring& key, const std::wstring& defaultValue, wchar_t* result, DWORD size);
    std::wstring ReadString(const std::wstring& section, const std::wstring& key, const std::wstring& defaultValue);

    void WriteInteger(const std::wstring& section, const std::wstring& key, int value);
    void WriteFloat(const std::wstring& section, const std::wstring& key, float value);
    void WriteBoolean(const std::wstring& section, const std::wstring& key, bool value);
    void WriteString(const std::wstring& section, const std::wstring& key, const std::wstring& value);
    void WriteDouble(const std::wstring& section, const std::wstring& key, double value);

    std::vector<float> ReadFloatArray(const std::wstring& section, const std::wstring& key);

private:
    std::wstring m_fileName;
};
