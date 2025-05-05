#pragma once
#include <string>
#include <fstream>

#if __has_include(<filesystem>)
#include <filesystem>
#define PANCAKE_FS_EXISTS
#else
#include <experimental/filesystem>
#endif

#include <nlohmann/json.hpp>

namespace pancake {
#ifdef PANCAKE_FS_EXISTS
    namespace fs = std::filesystem;
#else
    namespace fs = std::experimental::filesystem;
#endif

    static const pancake::fs::path s_RootDir = "/pancake/";
    static const pancake::fs::path s_ConfigDir = "config/";

    inline pancake::fs::path GetConfigDir() {
        pancake::fs::path rootDir = s_RootDir;
        if (!pancake::fs::is_directory(rootDir)) {
            rootDir = pancake::fs::current_path();
        }

        return rootDir / s_ConfigDir;
    }

    template <typename _Ty>
    inline bool LoadConfig(const std::string& name, _Ty& data) {
        auto filename = name + ".json";
        auto configPath = GetConfigDir() / filename;

        std::ifstream config(configPath);
        if (!config.is_open()) {
            return false;
        }

        nlohmann::json jsonData;
        config >> jsonData;
        config.close();

        jsonData.get_to(data);
        return true;
    }

    template <typename _Ty>
    inline void SaveConfig(const std::string& name, const _Ty& data) {
        auto filename = name + ".json";
        auto configDir = GetConfigDir();
        auto configPath = configDir / filename;

        if (!pancake::fs::is_directory(configDir)) {
            pancake::fs::create_directories(configDir);
        }

        std::ofstream config(configPath);
        config << nlohmann::json(data).dump(4);
        config.close();
    }
} // namespace pancake