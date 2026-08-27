#ifndef YAML_LOADER_HPP
#define YAML_LOADER_HPP

#if __cplusplus < 202002L
#error "This code requires C++20 or higher. Please set the compiler to use C++20."
#endif


#include <iostream>
#include <utility>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <vector>
#include <type_traits>

namespace yaml_loader {
    template <typename T>
    struct is_vector : std::false_type {};

    template <typename T, typename Alloc>
    struct is_vector<std::vector<T, Alloc>> : std::true_type {};


    class YamlLoader {
    public:
        explicit YamlLoader(const std::string& file_path)
            : path_(file_path), config_(YAML::LoadFile(path_)) {
            std::cout << "Load config file: " << path_ << std::endl;
        }

        explicit YamlLoader(YAML::Node document)
            : config_(std::move(document)) {
            if (!config_.IsDefined() || config_.IsNull()) {
                throw std::invalid_argument("YAML document must be defined and non-null");
            }
        }

        [[nodiscard]] const YAML::Node& document() const noexcept {
            return config_;
        }

        template <typename T>
        bool LoadParam(const std::string& param_name, T& param_value, const T& default_value = T{},
                       const bool& required = false) const {
            return loadParamInternal(param_name, param_value, default_value, required);
        }

        template <class T>
        bool LoadParam(const std::string& param_name, std::vector<T>& param_value,
                       const std::vector<T> default_value = {}, const bool& required = false) const {
            return loadParamInternal(param_name, param_value, default_value, required);
        }

    private:
        std::string path_;
        YAML::Node config_;

        template <typename T>
        bool loadParamInternal(const std::string& param_name, T& param_value, const T& default_value, bool required) const {
            YAML::Node tmp_node;
            if (containsSlash(param_name)) {
                tmp_node = getNodeFromPath(param_name);
            }
            else {
                const YAML::Node root = config_;
                if (root[param_name]) {
                    tmp_node = root[param_name];
                }
            }

            if (!tmp_node.IsDefined() ||
                tmp_node.Type() == YAML::NodeType::Undefined) {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
                param_value = default_value;
                printValue(param_value);
                if (required) {
                    throw std::invalid_argument("Required param " + param_name + " not found");
                }
                return false;
            }

            // A present value is part of the configuration contract.  Never
            // replace a malformed value with a default: that hides deployment
            // mistakes and can silently change safety behaviour.
            if (tmp_node.IsNull() ||
                (is_vector<T>::value && tmp_node.Type() != YAML::NodeType::Sequence)) {
                throw std::invalid_argument(
                    "Param " + param_name + " has an invalid type");
            }

            try {
                param_value = tmp_node.as<T>();
            } catch (const YAML::BadConversion&) {
                throw std::invalid_argument(
                    "Param " + param_name + " has an invalid type");
            }
            printf("\033[0;32m Load param %s success: \033[0;0m", param_name.c_str());
            printValue(param_value);
            return true;
        }

        static bool containsSlash(const std::string& str) {
            return str.find('/') != std::string::npos;
        }

        template <typename T>
        void printValue(const T& param_value) const {
            if constexpr (is_vector<T>::value) {
                std::cout << "[";
                for (const auto& elem : param_value) {
                    std::cout << elem << " ";
                }
                std::cout << "]" << std::endl;
            }
            else {
                std::cout << param_value << std::endl;
            }
        }

        [[nodiscard]] YAML::Node getNodeFromPath(const std::string& path) const {
            const YAML::Node root = config_;
            YAML::Node node = root;
            size_t pos = 0, next_pos;

            while ((next_pos = path.find('/', pos)) != std::string::npos) {
                std::string key = path.substr(pos, next_pos - pos);
                if (!node.IsDefined() ||
                    node.Type() == YAML::NodeType::Undefined) {
                    return YAML::Node(YAML::NodeType::Undefined);
                }
                if (!node.IsMap()) return YAML::Node(YAML::NodeType::Null);
                const YAML::Node child =
                        static_cast<const YAML::Node&>(node)[key];
                if (!child) return YAML::Node(YAML::NodeType::Undefined);
                node.reset(child);
                pos = next_pos + 1;
            }
            if (!node.IsDefined() || node.Type() == YAML::NodeType::Undefined) {
                return YAML::Node(YAML::NodeType::Undefined);
            }
            if (!node.IsMap()) return YAML::Node(YAML::NodeType::Null);
            const YAML::Node result = static_cast<const YAML::Node&>(node)[path.substr(pos)];
            return result ? result : YAML::Node(YAML::NodeType::Undefined);
        }
    };
}


#endif //YAML_LOADER_HPP
