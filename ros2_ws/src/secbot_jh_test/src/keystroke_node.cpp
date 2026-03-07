#include "keystroke_node.hpp"
// function needed for the threads
void keystroke_publisher::input_function() {
  std::cout << "helo from the input_function!\nType in a character:"
            << std::flush;
  while (rclcpp::ok()) {  // roughly equivalent to ESP_OK for error checking
    char_userinput =
        std::getchar();  // simple get character function for the non-canonical
                         // terminal we set up in main
    if (char_userinput != EOF) {
      auto message =
          std_msgs::msg::Char();  // same procedure in the talk_node.cpp make
                                  // the standard message
      message.data = char_userinput;                       // give it the input
      this->keystroke_publisher_object->publish(message);  // immediately
                                                           // publish
    }
  }
}
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);  // must always intialize this before anything else
  struct termios old_terminal_settings, new_terminal_settings;
  /*so it appears that every time we want to configure some settings for a UNIX
  terminal we can use something called termios(a structure) all we did here was
  simply toggle the bit that makes the termainl canonical(kinda the same way
  with MCUs and registers to determine if its an input or output)
  */
  tcgetattr(fileno(stdin), &old_terminal_settings);
  new_terminal_settings = old_terminal_settings;
  new_terminal_settings.c_lflag &= ~ICANON;
  new_terminal_settings.c_cc[VMIN] = 1;   // terminal blocks until at least ONE
                                          // character is placed in the stream
  new_terminal_settings.c_cc[VTIME] = 0;  // send immedatelly

  tcsetattr(fileno(stdin), TCSANOW,
            &new_terminal_settings);  // apply new terminal settings

  std::shared_ptr<keystroke_publisher> keystroke =
      std::make_shared<keystroke_publisher>();
  std::thread worker_thread(&keystroke_publisher::input_function, keystroke);
  rclcpp::spin(
      keystroke);  // probably don't need this but was placed for safe measures
  rclcpp::shutdown();
  /*
  rclcpp::spin is an infinite loop simply put it is a function doing what it
  needs to do for the ROS2 enviornment and its dwellers (specifically the
  callback functions we all know and love) placing the shutdown() closes that
  pointer? of keystroke and then the worker_thread can do its thing
  */
  if (worker_thread.joinable()) {
    worker_thread.join();  // makes the function input_function blocking meaning
                           // nothing gets sent UNTIL a keystroke is placed
  }
  tcsetattr(fileno(stdin), TCSANOW,
            &old_terminal_settings);  // if completely finished set the terminal
                                      // settings to what it had before..
  return 0;
}