#include "s21_stack.h"

namespace s21 {
template <typename T, template <typename> typename Container>
template <typename... Args>
void stack<T, Container>::insert_many_front(Args&&... args) {
  if (sizeof...(args) < 1) return;

  // std::initializer_list<value_type> values{std::forward<Args>(args)...};

  // for (const auto& value : values)
  //     Container_.push_front(value);

  (Container_.push_front(std::forward<Args>(args)), ...);
}
}  // namespace s21