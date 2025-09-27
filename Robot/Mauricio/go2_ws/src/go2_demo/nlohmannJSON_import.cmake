cmake_minimum_required(VERSION 3.14)

include(FetchContent)

FetchContent_Declare(json
	URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
)

FetchContent_MakeAvailable(json)
