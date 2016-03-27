# Lps
Code and files related to the Local Position System boards from Loligo (http://shop.loligo.se/products/store)

Installing with board manager
-----------------------------
Add the following url to the list of additional board managers under file/preferences: 
https://github.com/loligo/lps/raw/master/package_loligo_lps_index.json
Next, go to the Tools menu, Board and Boards Manager. Find Lps by Loligo and install. 

Installing without the board manager
------------------------------------
place a symlink in your arduino/hardware directory pointing to this directory. For example:
sudo ln -s ~/HOME/git/lps.git/ /usr/share/arduino/hardware/lps
