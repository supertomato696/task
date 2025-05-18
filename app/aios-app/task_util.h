#pragma once

#include <asio.hpp>
#include <future>

namespace bos::aios {
    template<typename R>
    inline R run_task_in_the_iocontext_sync(asio::io_context& io_context, std::function<R()> task) {
        if (io_context.get_executor().running_in_this_thread()) {
            return task();
        }

        std::promise<R> promise;
        auto future = promise.get_future();

        io_context.post([task, &promise]() {
            auto result = task();
            promise.set_value(result);
        });

        return future.get();
    }

    template<>
    inline void run_task_in_the_iocontext_sync<void>(asio::io_context& io_context, std::function<void()> task) {
        if (io_context.get_executor().running_in_this_thread()) {
            task();
            return;
        }

        std::promise<void> promise;
        std::future<void> future = promise.get_future();

        io_context.post([task, &promise]() {
            task();
            promise.set_value();
        });

        future.get();
    }

    inline void run_task_in_the_iocontext(asio::io_context& io_context, std::function<void()> task) {
        if (io_context.get_executor().running_in_this_thread()) {
            task();
            return;
        }

        io_context.post([task]() {
            task();
        });
    }

    inline void run_task_in_the_iocontext_async(asio::io_context& io_context, std::function<void()> task) {
        io_context.post([task]() {
            task();
        });
    }
}