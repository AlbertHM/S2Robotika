Contoh penggunaan (tested on CYGWIN) :

hasil compilasi 
netmaster.c --> netpc.exe
netroboard.c --> netrobot.exe

Contoh pemakaian
1. Pengiriman data dari roboard ke PC. PC sbg penerima dan Roboard sbg pengirim
   a. terminal1 jalankan ./netpc ru
   b. terminal2 jalankan ./netrobot tu 127.0.0.1 
   *) ru : mode receive
   *) tu : mode transceive
   *) 127.0.0.1 alamat IP yg mau dikirim
1. Pengiriman data dari PC ke roboard. PC sbg pengirim dan Roboard sbg penerima
   a. terminal1 jalankan ./netrobot ru
   b. terminal2 jalankan ./netpc tu 127.0.0.1 
   *) ru : mode receive
   *) tu : mode transceive
   *) 127.0.0.1 alamat IP yg mau dikirim
