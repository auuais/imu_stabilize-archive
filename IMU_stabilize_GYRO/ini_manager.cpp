#include <fstream>
#include <sstream>
#include <regex>
#include "ini_manager.h"

INIManager::INIManager(const std::wstring& fileName) : m_fileName(fileName)
{
    std::wifstream file(fileName);
    if (!file)
    {
        std::wofstream out(fileName);
        out.close();
    }
}

int INIManager::ReadInteger(const std::wstring& section, const std::wstring& key, int defaultValue)
{
    return GetPrivateProfileIntW(section.c_str(), key.c_str(), defaultValue, m_fileName.c_str());
}

float INIManager::ReadFloat(const std::wstring& section, const std::wstring& key, float defaultValue)
{
    wchar_t buffer[256];
    swprintf_s(buffer, L"%f", defaultValue);
    wchar_t result[256];
    GetPrivateProfileStringW(section.c_str(), key.c_str(), buffer, result, 256, m_fileName.c_str());
    return static_cast<float>(std::wcstod(result, nullptr));
}

double INIManager::ReadDouble(const std::wstring& section, const std::wstring& key, double defaultValue)
{
    wchar_t buffer[256];
    swprintf_s(buffer, L"%f", defaultValue);
    wchar_t result[256];
    GetPrivateProfileStringW(section.c_str(), key.c_str(), buffer, result, 256, m_fileName.c_str());
    return std::wcstod(result, nullptr);
}

bool INIManager::ReadBoolean(const std::wstring& section, const std::wstring& key, bool defaultValue)
{
    wchar_t buffer[256];
    swprintf_s(buffer, L"%s", defaultValue ? L"True" : L"False");
    wchar_t result[256];
    GetPrivateProfileStringW(section.c_str(), key.c_str(), buffer, result, 256, m_fileName.c_str());
    return (_wcsicmp(result, L"True") == 0 || _wcsicmp(result, L"true") == 0);
}

bool INIManager::ReadString(const std::wstring& section, const std::wstring& key, const std::wstring& defaultValue, wchar_t* result, DWORD size)
{
    return GetPrivateProfileStringW(section.c_str(), key.c_str(), defaultValue.c_str(), result, size, m_fileName.c_str()) > 0;
}

std::wstring INIManager::ReadString(const std::wstring& section, const std::wstring& key, const std::wstring& defaultValue)
{
    wchar_t buffer[1024] = { 0 };
    GetPrivateProfileStringW(section.c_str(), key.c_str(), defaultValue.c_str(), buffer, 1024, m_fileName.c_str());
    return std::wstring(buffer);
}

void INIManager::WriteInteger(const std::wstring& section, const std::wstring& key, int value)
{
    wchar_t buffer[64];
    swprintf_s(buffer, L"%d", value);
    WritePrivateProfileStringW(section.c_str(), key.c_str(), buffer, m_fileName.c_str());
}

void INIManager::WriteFloat(const std::wstring& section, const std::wstring& key, float value)
{
    wchar_t buffer[64];
    swprintf_s(buffer, L"%f", value);
    WritePrivateProfileStringW(section.c_str(), key.c_str(), buffer, m_fileName.c_str());
}

void INIManager::WriteDouble(const std::wstring& section, const std::wstring& key, double value)
{
    wchar_t buffer[64];
    swprintf_s(buffer, L"%f", value);
    WritePrivateProfileStringW(section.c_str(), key.c_str(), buffer, m_fileName.c_str());
}

void INIManager::WriteBoolean(const std::wstring& section, const std::wstring& key, bool value)
{
    WritePrivateProfileStringW(section.c_str(), key.c_str(), value ? L"True" : L"False", m_fileName.c_str());
}

void INIManager::WriteString(const std::wstring& section, const std::wstring& key, const std::wstring& value)
{
    WritePrivateProfileStringW(section.c_str(), key.c_str(), value.c_str(), m_fileName.c_str());
}

std::vector<float> INIManager::ReadFloatArray(const std::wstring& section, const std::wstring& key)
{
    wchar_t buffer[1024] = { 0 };
    GetPrivateProfileStringW(section.c_str(), key.c_str(), L"", buffer, 1024, m_fileName.c_str());

    std::wstring raw = buffer;

    // 중괄호 제거 및 쉼표 기준 분리
    raw.erase(std::remove(raw.begin(), raw.end(), L'{'), raw.end());
    raw.erase(std::remove(raw.begin(), raw.end(), L'}'), raw.end());

    std::wstringstream ss(raw);
    std::wstring token;
    std::vector<float> result;

    while (std::getline(ss, token, L',')) {
        try {
            float value = std::stof(token);
            result.push_back(value);
        }
        catch (...) {
            // 무시하거나 로그 가능
        }
    }

    return result;
}
