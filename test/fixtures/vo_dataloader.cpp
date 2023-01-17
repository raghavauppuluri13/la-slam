#include <la-slam/simpletest>

class MyFixture : TestFixture {
  public:
    void Setup() override { myInts.Add(1); }
    void TearDown() override { myInts.Reset(); }

    List<int> myInts;
}
