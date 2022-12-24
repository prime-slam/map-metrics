find_package(cilantro QUIET)

if (NOT cilantro_FOUND)
    message("Installing cilantro library")
    include(FetchContent)

    FetchContent_Declare(
        cilantro
        GIT_REPOSITORY https://github.com/kzampog/cilantro.git
        GIT_TAG cd80c1a5f2f4a9e2760fcc9b353ce0880f79783d # Jun 28, 2022
    )
    set(cilantro_BUILD_EXAMPLES OFF)

    FetchContent_MakeAvailable(cilantro)
endif()