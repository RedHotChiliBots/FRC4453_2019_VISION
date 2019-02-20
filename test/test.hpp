#include <unordered_map>

class TestResult {
private:
    const char* result;
    TestResult();
    TestResult(const char*);
public:
    static TestResult pass();
    static TestResult fail(const char*);

    bool passed() const;
    const char* msg() const;
};

extern const char* _TESTSUITE_NAME;
extern const std::unordered_map<const char*, TestResult (*)()> _TESTS;

#define TEST(name) TestResult _test_ ## name()

#define TEST_ASSERT(x, msg) \
if(!(x)) { \
    return TestResult::fail(msg); \
}

#define TEST_PASS return TestResult::pass();

#define START_TESTSUITE_REGISTRY(name) const char* _TESTSUITE_NAME=#name; const std::unordered_map<const char*, TestResult (*)()> _TESTS = {

#define REGISTER_TEST(name) {#name, _test_ ## name},

#define END_TESTSUITE_REGISTRY };