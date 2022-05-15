https://arduino.stackexchange.com/questions/54484/adding-a-custom-board-to-the-arduino-ide/60660#60660

The simplest clean way to add your board using an existing core is to add the variant to your sketches folder's hardware subfolder. Only boards.txt and files for the board variant are added.

In your sketches folder create a 'hardware' folder. In this folder create a folder with the name of your boards package and a subfolder with the architecture name and a subfolder for variant and a name of the variant. for example hardware/my_boards/samd/variants/samc21x

In your variant folder (variants/samc21x) put the files for your variant based on variant files of the closet variant in the referenced hardware package. In your case perhaps a mkrzero would be a good example.

In your package's root (hardware/my_boards/samd/) add boards.txt file with entries for your board.

For boards.txt take the options for a similar board in referenced package. Change the names and settings and prefix build.core value with the name of the vendor of the referenced package. For arduino/samd it would be arduino:. For example samc21x.build.core=arduino:arduino

My custom and customized boards definitions https://github.com/jandrassy/my_boards

Share
Improve this answer
Follow
edited Dec 12 '21 at 12:53

per1234
3,80322 gold badges1919 silver badges4040 bronze badges
answered Jan 15 '19 at 8:43

Juraj♦
15.6k33 gold badges2626 silver badges4141 bronze badges
and samc21x.build.extra_flags=-D__SAMC21G18A__ {build.usb_flags} or similar. the Arduino SAMD core supports compilation for SAMC – 
Juraj
♦
 Jan 16 '19 at 8:24
1
success story: forum.arduino.cc/index.php?topic=602377.msg4089655#msg4089655 – https://forum.arduino.cc/index.php?topic=602377.msg4089655#msg4089655

Juraj
♦
 Mar 9 '19 at