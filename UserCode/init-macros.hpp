/**
 * @file    init-macros.hpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#pragma once

#define new_(Type, ...)                                                                            \
    (                                                                                              \
            []() -> Type*                                                                          \
            {                                                                                      \
                static Type inst(__VA_ARGS__);                                                     \
                return &inst;                                                                      \
            }())
