add_library(json INTERFACE)

add_library(Nlohmann::Json ALIAS json)

target_include_directories(json
    INTERFACE
        ./
)


