#include <boost/test/unit_test.hpp>
#include <ugv_nav4d/Dummy.hpp>

using namespace ugv_nav4d;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    ugv_nav4d::DummyClass dummy;
    dummy.welcome();
}
