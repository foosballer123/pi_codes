#include <chrono>
#include <cppgpio.hpp>

using namespace GPIO;

int main()
{
  // use gpio #18

  DigitalOut out(10);
  
  // switch output to logical 1 (3.3V)

  out.on();
  
  // wait some time
  
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // switch it off again
  
  out.off();

  return 0;
}
