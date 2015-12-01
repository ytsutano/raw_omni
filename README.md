`raw_omni`: A Driver for Multiple Phantom Omni
==============================================

Since I've uploaded a few YouTube videos showing multiple Phantom Omnis running on a single ROS machine a couple of years ago (2013), many people have asked me to share the driver I wrote. I haven't polished the code for the following reasons:

- The driver was a quick hack just for private testing.
- I no longer have Phantom Omni devices to test.

However, I've got very polite emails from a couple of people for the driver. So I decided to upload the code as is.

## Launch Files

There are two launch files for `raw_omni`. You need to rewrite the serial numbers (printed on the bottom) to match your setup:

- raw_omni/launch/single_omni.launch
- raw_omni/launch/multi_omni.launch

## YouTube Videos

- [Reverse Engineered: Multiple Phantom Omni on Single ROS/Linux Machine](https://www.youtube.com/watch?v=l9I1vPg016k)
- [Automatic Homing](https://www.youtube.com/watch?v=zR56Btddc98)

## Author

- Yutaka Tsutano ([http://yutaka.tsutano.com](http://yutaka.tsutano.com))

## License

- See the LICENSE.md file for license rights and limitations (ISC).
