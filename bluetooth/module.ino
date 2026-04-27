bool printing = false;

template<typename T>
size_t print(const T& v) {
  if (!printing) return 0;
  return Serial.print(v);
}

// print(value, format) -> float decimals, int base, etc.
template<typename T>
size_t print(const T& v, int format) {
  if (!printing) return 0;
  return Serial.print(v, format);
}

// println(value)
template<typename T>
size_t println(const T& v) {
  if (!printing) return 0;
  return Serial.println(v);
}

// println(value, format)
template<typename T>
size_t println(const T& v, int format) {
  if (!printing) return 0;
  return Serial.println(v, format);
}

// println()
size_t println() {
  if (!printing) return 0;
  return Serial.println();
}
