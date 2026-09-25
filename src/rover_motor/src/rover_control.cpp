#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace {
int serial_fd = -1;
termios old_terminal{};
bool terminal_saved = false;
char current_command = 'S';

bool write_all(const std::string &s) {
  const char *p = s.data();
  size_t left = s.size();
  while (left > 0) {
    const ssize_t n = ::write(serial_fd, p, left);
    if (n < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    p += n;
    left -= static_cast<size_t>(n);
  }
  return true;
}

bool open_serial(const std::string &device) {
  serial_fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd < 0) return false;

  termios tty{};
  if (tcgetattr(serial_fd, &tty) != 0) {
    ::close(serial_fd);
    serial_fd = -1;
    return false;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

  if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
    ::close(serial_fd);
    serial_fd = -1;
    return false;
  }
  return true;
}

void send_movement(char command) {
  current_command = command;
  write_all(std::string(1, command) + "\n");
}

void restore_terminal() {
  if (serial_fd >= 0) write_all("S\n");
  if (terminal_saved) tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal);
  if (serial_fd >= 0) {
    ::close(serial_fd);
    serial_fd = -1;
  }
}

bool raw_terminal() {
  if (tcgetattr(STDIN_FILENO, &old_terminal) != 0) return false;
  terminal_saved = true;
  termios raw = old_terminal;
  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  return tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
}

void set_pwm(int left, int right) {
  write_all("A" + std::to_string(left) + "\n");
  usleep(30000);
  write_all("C" + std::to_string(right) + "\n");
}
}  // namespace

int main(int argc, char **argv) {
  const std::string device = argc > 1 ? argv[1] : "/dev/ttyUSB0";

  if (!open_serial(device)) {
    std::cerr << "Could not open " << device << ": " << std::strerror(errno) << "\n";
    return 1;
  }

  if (!raw_terminal()) {
    std::cerr << "Could not configure terminal\n";
    ::close(serial_fd);
    return 1;
  }

  std::atexit(restore_terminal);

  // Opening the USB serial port can reset the ESP32.
  usleep(1500000);

  int left_pwm = 120;
  int right_pwm = 120;
  set_pwm(left_pwm, right_pwm);
  send_movement('S');

  std::cout << "ROVERBOT manual control\n"
            << "W forward | S backward | A left | D right | SPACE stop\n"
            << "[ ] left PWM -/+5 | - = right PWM -/+5 | P show | Q quit\n"
            << "PWM: L=" << left_pwm << " R=" << right_pwm << std::endl;

  auto last_refresh = std::chrono::steady_clock::now();

  while (true) {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(STDIN_FILENO, &read_set);

    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000;

    const int ready = select(STDIN_FILENO + 1, &read_set, nullptr, nullptr, &timeout);
    if (ready < 0) {
      if (errno == EINTR) continue;
      std::cerr << "Keyboard select failed: " << std::strerror(errno) << "\n";
      return 1;
    }

    if (ready > 0 && FD_ISSET(STDIN_FILENO, &read_set)) {
      char key = 0;
      if (::read(STDIN_FILENO, &key, 1) == 1) {
        switch (key) {
          case 'w': case 'W': send_movement('F'); break;
          case 's': case 'S': send_movement('B'); break;
          case 'a': case 'A': send_movement('L'); break;
          case 'd': case 'D': send_movement('R'); break;
          case ' ': send_movement('S'); break;
          case '[':
            left_pwm = std::max(0, left_pwm - 5);
            set_pwm(left_pwm, right_pwm);
            std::cout << "PWM: L=" << left_pwm << " R=" << right_pwm << std::endl;
            break;
          case ']':
            left_pwm = std::min(255, left_pwm + 5);
            set_pwm(left_pwm, right_pwm);
            std::cout << "PWM: L=" << left_pwm << " R=" << right_pwm << std::endl;
            break;
          case '-':
            right_pwm = std::max(0, right_pwm - 5);
            set_pwm(left_pwm, right_pwm);
            std::cout << "PWM: L=" << left_pwm << " R=" << right_pwm << std::endl;
            break;
          case '=':
            right_pwm = std::min(255, right_pwm + 5);
            set_pwm(left_pwm, right_pwm);
            std::cout << "PWM: L=" << left_pwm << " R=" << right_pwm << std::endl;
            break;
          case 'p': case 'P':
            std::cout << "PWM: L=" << left_pwm << " R=" << right_pwm << std::endl;
            break;
          case 'q': case 'Q':
            send_movement('S');
            return 0;
          default:
            break;
        }
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_refresh).count() >= 200) {
      if (current_command != 'S') write_all(std::string(1, current_command) + "\n");
      last_refresh = now;
    }
  }
}
