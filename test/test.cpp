#include "test.hpp"
#include <iostream>
#include <vector>

int main() {
    size_t num_tests = _TESTS.size();

    std::vector<std::pair<const char*, TestResult>> failed_tests;

    std::cout << "Running test suite \"" << _TESTSUITE_NAME << "\"..." << std::endl;

    if(failed_tests.size() != 1)
    {
        std::cout << "Failures: " << std::endl;
    } else {
        std::cout << "Failure: " << std::endl;
    }

    for(const auto& i : _TESTS) {
        std::cout << i.first << "...";
        std::cout.flush();
        TestResult res = (i.second)();
        if (!res.passed()) {
            std::cout << "FAILED" << std::endl;
            failed_tests.push_back(std::make_pair(i.first, res));
        }
        else {
            std::cout << "PASSED" << std::endl;
        }
    }

    std::cout << "Result: " << num_tests - failed_tests.size() << "/" << num_tests << " tests passed." << std::endl;

    if(!failed_tests.empty()) {
        if(failed_tests.size() != 1)
        {
            std::cout << "Failures: " << std::endl;
        } else {
            std::cout << "Failure: " << std::endl;
        }
        
        for(const auto& f : failed_tests)
        {
            std::cout << "    Test " << f.first << " failed with message \"" << f.second.msg() << "\"" << std::endl;
        }
        
        if(failed_tests.size() != 1)
        {
            std::cout << failed_tests.size() << " failed tests!" << std::endl;
        } else {
            std::cout << failed_tests.size() << " failed test!" << std::endl;
        }
        return -1;
    }

    return 0;
}

TestResult::TestResult() : result(nullptr) {}
TestResult::TestResult(const char* r) : result(r) {}

TestResult TestResult::fail(const char* r) {
    return TestResult(r);
}

TestResult TestResult::pass() {
    return TestResult();
}

bool TestResult::passed() const {
    return result == nullptr;
}

const char* TestResult::msg() const {
    if(result) {
        return result;
    }
    return "N/A";
}