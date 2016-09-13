//          Copyright Felipe Magno de Almeida 2016.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef AVR_ADD_AVR_ADD_HPP
#define AVR_ADD_AVR_ADD_HPP

#include <avr-traits/adc.hpp>
#include <avr/interrupt.h>

#include <avr-c++11-workaround/forward.hpp>

#include <avr-add/detail/config.hpp>

#include <assert.h>

namespace avr_add {

namespace detail {
  
typedef void(*function_t)(int value, void const* data);
typedef void(*free_function_t)(void const* data);

struct port_registration
{
  function_t function;
  void const* data;
  free_function_t free;

  explicit operator bool() const volatile { return !!function; }
  port_registration volatile& operator=(port_registration const& other) volatile
  {
    if(free)
      free(data);
    function = other.function;
    data = other.data;
    free = other.free;
  }
};
static volatile port_registration table[avr_traits::adc_ports];
int volatile current_port = -1;
bool volatile old_port_in_progress = false;
int volatile next_port = -1;
  
template <typename F>
void invoke(int value, void const* data)
{
  (*static_cast<F const*>(data))(value);
}

template <typename F>
void free(void const* data)
{
  delete static_cast<F const*>(data);
}

inline void start_conversion()
{
  AVR_ADD_ADCSRA |= (1 << AVR_ADD_ADSC);
}

inline void stop_conversion()
{
  AVR_ADD_ADCSRA &= ~(1 << AVR_ADD_ADSC);
}
  
inline void push_port_start(int port)
{
  if(current_port == -1)
  {
    avr_traits::adc_port_set_admux(port);
    current_port = port;
    start_conversion();
  }
}
  
}

void init()
{
   // initialize adc 
   // AREF = AVcc
   AVR_ADD_ADMUX = (1<<AVR_ADD_REFS0);
 
   // ADC Enable and prescaler of 128
   // 16000000/128 = 125000
   AVR_ADD_ADCSRA = (1<<AVR_ADD_ADEN)|(1<<AVR_ADD_ADPS2)|(1<<AVR_ADD_ADPS1)|(1<<AVR_ADD_ADPS0);

   // No MUX values needed to be changed to use ADC0

   // Set ADC to Free-Running Mode
   AVR_ADD_ADCSRA |= (1 << AVR_ADD_ADATE);

   // Enable ADC Interrupt
   AVR_ADD_ADCSRA |= (1 << AVR_ADD_ADIE);
}
  
template <typename F>
void register_port(int port, F&& f)
{
  typedef typename std::remove_reference<F>::type function_object_type;
  detail::table[port] =
    {
      static_cast<detail::function_t>(&detail::invoke<function_object_type>)
      , new function_object_type(std::forward<F>(f))
      , static_cast<detail::free_function_t>(&detail::free<function_object_type>)
    };
  detail::push_port_start(port);
}

void unregister_port(int port)
{
  detail::table[port] = {};
  // not exactly
  avr_traits::adc_port_reset_admux();
}
  
template <typename T>
struct static_add
{
    template <typename F0>
    static_add(F0 f0)
    {
      avr_add::register_port(T::adc_pin, f0);
    }

    ~static_add()
    {
      avr_add::unregister_port(T::adc_pin);
    }
};

namespace detail {
  
ISR(ADC_vect)
{
    int current_port = detail::current_port;
    {
      volatile int value = ADC;
      if(detail::table[current_port])
        detail::table[current_port].function(value, detail::table[current_port].data);
    }

    int next_port = -1;
    if(old_port_in_progress) // already reading detail::next_port,
                             // must find the next of the next
      current_port = detail::next_port;

    // from current to 
    for(unsigned int i = current_port+1; i != avr_traits::adc_ports; i++)
      if(detail::table[i])
      {
        next_port = i;
        break;
      }
    
    if(next_port == -1)
      for(int i = 0; i != current_port + 1; i++)
        if(detail::table[i])
        {
          next_port = i;
          break;
        }

    if(old_port_in_progress)
    {
      assert(detail::next_port != -1);
      // reading of detail::next_port already in progress
      if(AVR_ADD_ADCSRA & ADSC)
      {
        detail::current_port = detail::next_port;
        detail::next_port = next_port;
        if(next_port != -1)
          avr_traits::adc_port_set_admux(next_port);
      }
      else
      {
        // detail::next_port hasn't started being read yet. Wait for next interruption
        // with next as current
        detail::current_port = detail::next_port;
        detail::next_port = -1;
        old_port_in_progress = false;
      }
    }
    else if(next_port != -1)
    {
      assert(detail::next_port == -1);
      avr_traits::adc_port_set_admux(next_port);
      // next call will have value from next_port
      if(!(AVR_ADD_ADCSRA & ADSC))
        {
          detail::current_port = next_port;
          detail::next_port = -1;
        }
      else // already reading last port, next value will have
           // value from current port
        {
          old_port_in_progress = true;
          detail::next_port = next_port;
          avr_traits::adc_port_set_admux(next_port);
        }
    }
    else
      stop_conversion();
}

} }
  
#endif
