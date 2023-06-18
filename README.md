# Beşiktaş Rsports Task
2 Adet falcon 500 motor ve talonfx motor kontrolcü kullanılıyor, robot kolun açısını PID+F değerleri ile istenilen açıya getiriyor.

Robot kolun derecesini ayarlamak için constants içerisindeki m_armSetpointDegrees değerini değiştirebilirsiniz, PID+F değerleri tam stabil değil. Kolun ağırlığı 8kg, istenilen derece ile arasındaki hata simülasyonda açık olan plot üzerinde ya da networktables'da "position error" olarak görülebilir.
