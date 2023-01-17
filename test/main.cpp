#include <la-slam/common.h>
#include <la-slam/simpletest.h>

using namespace std;

char const *groups[] = {"VO"};

int main() {
    bool pass = true;

    for (auto group : groups)
        pass &= TestFixture::ExecuteTestGroup(group, TestFixture::Verbose);

    return pass ? 0 : 1;
}
