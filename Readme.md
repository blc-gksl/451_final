
Çalıştırmadan önce txt dosyasının adresini girmek gerekiyor. main.cpp de 763. satır, objSpawner.cpp dosyasında 168 satıra txt file'ın adresini belirtmeniz gerekiyor.

Simulasyonu ve objSpawner nodunu çalıştırır:



$ roslaunch tgr_simulation tgr_start.launch 

daha sonra main nodu çalıştırarak aracı hareket ettiriyoruz. Arac 0,0 noktasında spawnlanıyor ve 0,0 - 50,50 alanında objeler spawn ediyor ve bu alanda çalışıyor. 


$ rosrun tgr_simulation main


