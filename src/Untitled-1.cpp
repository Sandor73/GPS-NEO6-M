/* Программа полностью написана с помощью нейросети PHIND,
 * которая работает на базе ГПТ-4(Генеративный Предобученный Трансформер)
 *
 * Программа для работы с GPS модулем и отображения информации на дисплее.
 * Функции программы:
 *  1. Инициализация дисплея и GPS модуля.
 *  2. Обработка данных GPS: чтение данных из последовательного порта, проверка валидности данных.
 *  3. Отображение информации о GPS на дисплее: время, дата, основные данные GPS (широта, долгота, скорость, высота), вторичные данные GPS (количество спутников, HDOP).
 *  4. Обработка нажатия кнопки: переключение между меню, отображение информации о GPS.
 *  5. Анимация изображений на дисплее при ожидании обновления данных GPS.
 *  6. Обработка сигнала потери связи с GPS модулем: генерация звукового сигнала, отображение сообщения на дисплее.
 *  7. Отображение сообщения о потере сигнала на дисплее с задержкой, чтобы дать время на обработку данных GPS.
 *  8. Переключение между различными состояниями меню в зависимости от нажатия кнопки.
 *  9. Отображение информации о GPS в зависимости от текущего состояния меню.
 *  10. Обработка ситуации, когда данные GPS недоступны: отображение сообщения на дисплее и возможность переключения между меню.
 */
#include <HardwareSerial.h>
#include <Arduino.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include "splash_image.h"
#include "splash_image_1.h"

U8G2_ST7565_ERC12864_ALT_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/15, /* data=*/2, /* cs=*/5, /* dc=*/4, /* reset=*/16); // for ESP32  contrast improved version for ERC12864
// U8G2_ST7565_ERC12864_ALT_F_4W_SW_SPI u8g2(U8G2_R0, /* scl=*/13, /* si=*/11, /* cs=*/10, /* rs=*/9, /* rse=*/8);
// NEO-6m - RXPin = 1, TXPin = 3;

static const uint32_t GPSBaud = 9600;

const int buzzerPin = 12;  // Пин для генерации звука
const int buttonPin = 13;  // Определение пина для кнопки
bool signalPlayed = false; // Глобальная переменная для отслеживания того, был ли сгенерирован сигнал
bool gpsUpdated = false;   // Глобальная переменная для отслеживания обновления GPS
int menuState = 0;         // Переменная для отслеживания текущего состояния меню
unsigned long lastButtonPressTime = 0;// Время последнего нажатия кнопки. Используется для отслеживания времени между нажатиями кнопки.
bool buttonPressed = false; // Флаг, указывающий, была ли кнопка нажата. Используется для определения, когда кнопка была нажата и когда она была отпущена.

TinyGPSPlus gps; // Создаем объект gps

/*Метод вызова объектов библиотеки TinyGPSPlus
u8g2.print(gps.location.lat(),  6); // Выводим широту в градусах (двойное число)
u8g2.print(gps.location.lng(),  6); // Выводим долготу в градусах (двойное число)
u8g2.print(gps.location.rawLat().negative ? "-" : "+"); // Выводим знак широты
u8g2.print(gps.location.rawLat().deg); // Выводим целую часть широты
u8g2.print(gps.location.rawLat().billionths); // Выводим миллиарды широты (u16/u32)
u8g2.print(gps.location.rawLng().negative ? "-" : "+"); // Выводим знак долготы
u8g2.print(gps.location.rawLng().deg); // Выводим целую часть долготы
u8g2.print(gps.location.rawLng().billionths); // Выводим миллиарды долготы (u16/u32)
u8g2.print(gps.date.value()); // Выводим сырой дату в формате ДДММГГ (u32)
u8g2.print(gps.date.year()); // Выводим год (2000+) (u16)
u8g2.print(gps.date.month()); // Выводим месяц (1-12) (u8)
u8g2.print(gps.date.day()); // Выводим день (1-31) (u8)
u8g2.print(gps.time.value()); // Выводим сырое время в формате ЧЧММСССС (u32)
u8g2.print(gps.time.hour()); // Выводим час (0-23) (u8)
u8g2.print(gps.time.minute()); // Выводим минуты (0-59) (u8)
u8g2.print(gps.time.second()); // Выводим секунды (0-59) (u8)
u8g2.print(gps.time.centisecond()); // Выводим сотые доли секунды (0-99) (u8)
u8g2.print(gps.speed.value()); // Выводим сырую скорость в сотых долях узла (i32)
u8g2.print(gps.speed.knots()); // Выводим скорость в узлах (двойное число)
u8g2.print(gps.speed.mph()); // Выводим скорость в милях в час (двойное число)
u8g2.print(gps.speed.mps()); // Выводим скорость в метрах в секунду (двойное число)
u8g2.print(gps.speed.kmph()); // Выводим скорость в километрах в час (двойное число)
u8g2.print(gps.course.value()); // Выводим сырой курс в сотых долях градуса (i32)
u8g2.print(gps.course.deg()); // Выводим курс в градусах (двойное число)
u8g2.print(gps.altitude.value()); // Выводим сырую высоту в сантиметрах (i32)
u8g2.print(gps.altitude.meters()); // Выводим высоту в метрах (двойное число)
u8g2.print(gps.altitude.miles()); // Выводим высоту в милях (двойное число)
u8g2.print(gps.altitude.kilometers()); // Выводим высоту в километрах (двойное число)
u8g2.print(gps.altitude.feet()); // Выводим высоту в футах (двойное число)
u8g2.print(gps.satellites.value()); // Выводим количество используемых спутников (u32)
u8g2.print(gps.hdop.value()); // Выводим горизонтальную точность измерения (сотые доли-i32)
*/

// Функция для отображения изображения анимации
void displayImage(const uint8_t *imageData)
{
    u8g2.firstPage();
    do
    {
        u8g2.drawBitmap(0, 0, image_width, image_height, imageData);
        u8g2.drawBitmap(0, 0, image_1_width, image_1_height, imageData);
        u8g2.setFont(u8g2_font_6x13_t_cyrillic); // Устанавливаем шрифт для дисплея
        // Выводим текст "GPS" и "Навигатор" на дисплей
        u8g2.setCursor(image_width + 0, 10); // Позиционируем курсор справа от изображения спутника
        u8g2.print("GPS");
        u8g2.setCursor(image_width + 0, 20); // Позиционируем курсор справа от изображения спутника
        u8g2.print("Навигатор");
        // Выводим текст "GPS" и "Навигатор" на дисплей
        u8g2.setCursor(image_1_width + 0, 10); // Позиционируем курсор справа от изображения спутника
        u8g2.print("GPS");
        u8g2.setCursor(image_1_width + 0, 20); // Позиционируем курсор справа от изображения спутника
        u8g2.print("Навигатор");
        tone(buzzerPin, 10, 50); // Генерируем звуковой сигнал на частоте  1000 Гц в течение  500 мс

    } while (u8g2.nextPage());
}
// Функция анимации
void animateImage()
{
    const uint8_t *images[] = {image, image_1 /*, дополнительные изображения, е��ли есть */};
    const size_t numImages = sizeof(images) / sizeof(images[0]);
    const unsigned long frameDelay = 500; // Задержка между кадрами в миллисекундах
    unsigned long startTime = millis();   // Запоминаем время начала анимации

    while (!gpsUpdated && (millis() - startTime) < 5000) // Анимация продолжается до тех пор, пока не обновлен GPS и не прошло  5 секунд
    {
        for (size_t i = 0; i < numImages; i++)
        {
            displayImage(images[i]);
            delay(frameDelay);
        }
        // Проверяем, обновлена ли информация о местоположении
        if (gps.location.isUpdated())
        {
            gpsUpdated = true; // Устанавливаем флаг, что GPS обновлен
            break;             // Выходим из цикла анимации
        }
    }
}

void setup()
{
    u8g2.begin();         // Инициализируем дисплей
    u8g2.setContrast(90); // Устанавли��аем контраст дисплея
    Serial.begin(9600);   // Начинаем последовательное соединение с GPS

    u8g2.clearBuffer();                      // Очищаем буфер дисплея
    u8g2.enableUTF8Print();                  // Включаем поддержку UTF-8 символов
    u8g2.setFont(u8g2_font_6x13_t_cyrillic); // Устанавливаем шрифт для дисплея

    // Рисуем битмап на дисплее
    u8g2.drawBitmap(0, 0, image_width, image_height, image);
    u8g2.drawBitmap(0, 0, image_1_width, image_1_height, image_1);

    pinMode(buttonPin, INPUT_PULLUP); // Настраиваем пин кнопки на чтение с подтяжкой к питанию

    animateImage();
}

// Функция для отображения времени и даты
void displayTimeAndDate()
{
    u8g2.clearBuffer();
    u8g2.enableUTF8Print();                   // Включаем поддержку UTF-8 символов
    u8g2.setFont(u8g2_font_10x20_t_cyrillic); // Устанавливаем большой шрифт для времени и даты
    if (gps.time.isValid() && gps.date.isValid())
    {
        u8g2.setCursor(5, 20); // Позиционируем курсор на дисплее
        // u8g2.print("Время   :");                       // Выводим текст на дисплей
        u8g2.print(gps.time.hour() < 10 ? "0" : "");   // Добавляем ведущий ноль, если час меньше  10
        u8g2.print(gps.time.hour());                   // Выводим часы
        u8g2.print(":");                               // Выводим разделитель
        u8g2.print(gps.time.minute() < 10 ? "0" : ""); // Добавляем ведущий ноль, если минута меньше  10
        u8g2.print(gps.time.minute());                 // Выводим минуты
        u8g2.print(":");                               // Выводим разделитель
        u8g2.print(gps.time.second() < 10 ? "0" : ""); // Добавляем ведущий ноль, если секунда меньше  10
        u8g2.print(gps.time.second());                 // Выводим секунды
        u8g2.print(" UTC");

        u8g2.setCursor(13, 50); // Устанавливаем позицию курсора на дисплее
        // u8g2.print("Дата    :");                // Выводим текст на дисплей
        u8g2.print(gps.date.year()); // Выводим год
        u8g2.print("-");             // Выводим разделитель
        if (gps.date.month() < 10)
            u8g2.print("0");          // ��обавляем ведущий ноль, если месяц меньше 10
        u8g2.print(gps.date.month()); // Выводим месяц
        u8g2.print("-");              // Выводим раздели��ель
        if (gps.date.day() < 10)
            u8g2.print("0");        // Добавляем ведущий ноль, если день меньше 10
        u8g2.print(gps.date.day()); // Выводим день
    }
    else
    {
        u8g2.setCursor(5, 25);      // Позиционируем курсор на дисплее
        u8g2.print("ВРЕМЯ И ДАТА"); // Выводим текст на дисплее
        u8g2.setCursor(15, 45);     // Позиционируем курсор на дисплее
        u8g2.print("НЕДОСТУПНЫ");   // Выводим текст на дисплее
    }
    u8g2.sendBuffer(); // Отправляем буфер на дисплей
}

// Функция для отображения основных данных GPS
void displayMainGPSData()
{
    u8g2.clearBuffer();
    u8g2.enableUTF8Print();                  // Включаем поддержку UTF-8 символов
    u8g2.setFont(u8g2_font_6x13_t_cyrillic); // Устанавливаем шрифт для основных данных

    // Проверяем, доступна ли информация о местоположении
    if (gps.date.isValid())
    {
        u8g2.setCursor(0, 10);             // Позиционируем курсор на дисплее
        u8g2.print("Долгота :");           // Выводим текст на дисплей
        u8g2.print(gps.location.lng(), 6); // Выводим долготу

        u8g2.setCursor(0, 20);             // Позиционируем курсор на дисплее
        u8g2.print("Широта  :");           // Выводим текст на дисплей
        u8g2.print(gps.location.lat(), 6); // Выводим широту
    }
    else
    {
        u8g2.setCursor(0, 10);              // Позиционируем ку��сор на дисплее
        u8g2.print("Локация : Недоступно"); // Выводим текст на дисплее
    }

    // Проверяем, доступна ли информация о высоте
    if (gps.date.isValid())
    {
        u8g2.setCursor(0, 30);             // Позиционируем курсор на дисплее
        u8g2.print("Высота  :");           // Выводим текст на дисплей
        u8g2.print(gps.altitude.meters()); // Выводим высоту над уровнем моря
    }
    else
    {
        u8g2.setCursor(0, 30);              // Позиционируем ку��сор на дисплее
        u8g2.print("Высота  : Недоступно"); // Выводим текст на дисплее
    }

    // Проверяем, доступна ли информац��я о скорости
    if (gps.date.isValid())
    {
        u8g2.setCursor(0, 40);           // Позиционируем курсор на дисплее
        u8g2.print("Скорость:");         // Выводим текст на дисплей
        u8g2.print(gps.speed.kmph(), 1); // Выводим скорость в км/ч с одним знаком после запятой
    }
    else
    {
        u8g2.setCursor(0, 40);              // Позиционируем курсор на дисплее
        u8g2.print("Скорость: Недоступно"); // Выводим текст на дисплее
    }

    // Проверяем, доступна ли информация о дате
    if (gps.date.isValid() && gps.location.isValid())
    {
        u8g2.setCursor(0, 50);       // Устанавливаем позицию курсора на дисплее
        u8g2.print("Дата    :");     // Выводим текст на дисплей
        u8g2.print(gps.date.year()); // Выводим год
        u8g2.print("-");             // Выводим разделитель
        if (gps.date.month() < 10)
            u8g2.print("0");          // ��обавляем ведущий ноль, если месяц меньше 10
        u8g2.print(gps.date.month()); // Выводим месяц
        u8g2.print("-");              // Выводим раздели��ель
        if (gps.date.day() < 10)
            u8g2.print("0");        // Добавляем ведущий ноль, если день меньше 10
        u8g2.print(gps.date.day()); // Выводим день
    }
    else
    {
        u8g2.setCursor(0, 50);              // Устанавливаем позицию курсора на дисплее
        u8g2.print("Дата    : Недоступно"); // Выводим текст на дисплей
    }

    // Проверяем, доступна ли информация о времени
    if (gps.date.isValid() && gps.location.isValid())
    {
        u8g2.setCursor(0, 60);                         // Позиционируем курсор на дисплее
        u8g2.print("Время   :");                       // Выводим текст на дисплей
        u8g2.print(gps.time.hour() < 10 ? "0" : "");   // Добавляем ведущий ноль, если час меньше  10
        u8g2.print(gps.time.hour());                   // Выводим часы
        u8g2.print(":");                               // Выводим разделитель
        u8g2.print(gps.time.minute() < 10 ? "0" : ""); // Добавляем ведущий ноль, если минута меньше  10
        u8g2.print(gps.time.minute());                 // Выводим минуты
        u8g2.print(":");                               // Выводим разделитель
        u8g2.print(gps.time.second() < 10 ? "0" : ""); // Добавляем ведущий ноль, если секунда меньше  10
        u8g2.print(gps.time.second());                 // Выводим секунды
        u8g2.print(" UTC");
    }
    else
    {
        u8g2.setCursor(0, 60);              // Позиционируем курсор н�� дисплее
        u8g2.print("Время   : Недоступно"); // Выводим текст на дисплее
    }
    u8g2.sendBuffer(); // Отправляем буфер на дисплей
}

// Функция для отображения вторичных данных GPS
void displaySecondaryGPSData()
{
    u8g2.clearBuffer();
    u8g2.enableUTF8Print();                  // Включаем поддержку UTF-8 символов
    u8g2.setFont(u8g2_font_6x13_t_cyrillic); // Устанавливаем шрифт для вторичных данных
    if (gps.satellites.isValid() && gps.hdop.isValid())
    {
        u8g2.setCursor(0, 10);
        u8g2.print("Спутники: ");
        u8g2.print(gps.satellites.value(), DEC);
        u8g2.setCursor(0, 20);
        u8g2.print("HDOP    : ");
        u8g2.print(gps.hdop.value(), DEC);
        // Добавьте здесь другие вторичные данные, если нужно
        u8g2.sendBuffer();
    }
    else
    {
        u8g2.setCursor(20, 20);   // Позиционируем курсор на дисплее
        u8g2.print("Sat/HDOP: "); // Выводим текст на дисплее

        u8g2.setCursor(20, 50);   // Позиционируем курсор на дисплее
        u8g2.print("Нет данных"); // Выводим текст на дисплее
    }
    u8g2.sendBuffer(); // Отправляем буфер на дисплей
}

//  Функция кнопки
void buttonPinRead()
{
    static unsigned long lastDebounceTime = 0; // Время последнего отсчета дребезга
    const unsigned long debounceDelay = 50;    // Задержка для отсчета дребезга

    // Читаем состояние кнопки
    bool buttonState = digitalRead(buttonPin) == LOW;

    // Если кнопка нажата и прошло достаточно времени с момента последнего отсчета дребезг��
    if (buttonState && !buttonPressed && millis() - lastDebounceTime > debounceDelay)
    {
        lastButtonPressTime = millis(); // Запоминаем время нажатия
        buttonPressed = true;           // Устанавливаем флаг, что кнопка нажата
    }

    // Если кнопка отпущена и прошло достаточно времени с момента последнего нажатия
    if (!buttonState && buttonPressed && millis() - lastButtonPressTime > debounceDelay)
    {

        menuState = (menuState + 1) % 3; // Переключаем состояние меню
        switch (menuState)
        {
        case 0:
            displayTimeAndDate(); // Отображаем время и дату
            break;
        case 1:
            displayMainGPSData(); // Отображаем основные данные GPS
            break;
        case 2:
            displaySecondaryGPSData(); // Отображаем вторичные данные GPS
            break;
        }
        u8g2.clearBuffer();    // Очищаем буфер дисплея
        u8g2.sendBuffer();     // Отправляем буфер на дисплей
        buttonPressed = false; // Сбрасываем флаг, что кнопка отпущена
    }

    // Обновляем время последнего отсчета дребезга
    if (buttonState != buttonPressed)
    {
        lastDebounceTime = millis();
    }
}

/*//Функция отображения на дисплее без управления экранами кнопкой
void displayInfo()
{
  u8g2.clearBuffer();                      // Очищаем буфер дисплея
  u8g2.enableUTF8Print();                  // Включаем поддержку UTF-8 символов
  u8g2.setFont(u8g2_font_6x13_t_cyrillic); // Устанавливаем шрифт для дисплея

  // Проверяем, доступна ли ин��ормация о количестве спутников
  if (gps.satellites.isValid())
  {
    u8g2.setCursor(95, 10);             // Позиционируем курсор на дисплее
    u8g2.print("Sat:");                 // Выводим текст на дисплей
    u8g2.print(gps.satellites.value()); // Выводим количество спутников
  }
  else
  {
    u8g2.setCursor(95, 10); // Позиционируем курсор на дисплее
    u8g2.print("Sat:-");   // Выводим текст на дисплее
  }

  // Проверяем, доступна ли информация о местоположении
  if (gps.location.isUpdated())
  {
    u8g2.setCursor(0, 30);             // Позиционируем курсор на дисплее
    u8g2.print("Широта  :");           // Выводим текст на дисплей
    u8g2.print(gps.location.lat(), 6); // Выводим широту

    u8g2.setCursor(0, 20);             // Позиционируем курсор на дисплее
    u8g2.print("Долгота :");           // Выводим текст на дисплей
    u8g2.print(gps.location.lng(), 6); // Выводим долготу
  }
  else
  {
    u8g2.setCursor(0, 10);              // Позиционируем ку��сор на дисплее
    u8g2.print("Локация : Недоступно"); // Выводим текст на дисплее
  }

  // Проверяем, доступна ли информация о высоте
  if (gps.altitude.isValid())
  {
    u8g2.setCursor(0, 10);             // Позиционируем курсор на дисплее
    u8g2.print("Высота  :");           // Выводим текст на дисплей
    u8g2.print(gps.altitude.meters()); // Выводим высоту над уровнем моря
    u8g2.print(";");                   // Выводим текст на дисплей
  }
  else
  {
    u8g2.setCursor(0, 10);              // Позиционируем ку��сор на дисплее
    u8g2.print("Локация : Недоступно"); // Выводим текст на дисплее
  }

  // Проверяем, доступна ли информац��я о скорости
  if (gps.speed.isValid())
  {
    u8g2.setCursor(0, 40);           // Позиционируем курсор на дисплее
    u8g2.print("Скорость:");         // Выводим текст на дисплей
    u8g2.print(gps.speed.kmph(), 1); // Выводим скорость в км/ч с одним знаком после запятой
  }
  else
  {
    u8g2.setCursor(0, 40);              // Позиционируем курсор на дисплее
    u8g2.print("Скорость: Недоступна"); // Выводим текст на дисплее
  }

  // Проверяем, доступна ли информация о дате
  if (gps.date.isValid())
  {
    u8g2.setCursor(0, 50);       // Устанавливаем позицию курсора на дисплее
    u8g2.print("Дата    :");     // Выводим текст на дисплей
    u8g2.print(gps.date.year()); // Выводим год
    u8g2.print("-");             // Выводим разделитель
    if (gps.date.month() < 10)
      u8g2.print("0");            // ��обавляем ведущий ноль, если месяц меньше 10
    u8g2.print(gps.date.month()); // Выводим месяц
    u8g2.print("-");              // Выводим раздели��ель
    if (gps.date.day() < 10)
      u8g2.print("0");          // Добавляем ведущий ноль, если день меньше 10
    u8g2.print(gps.date.day()); // Выводим день
  }
  else
  {
    u8g2.setCursor(0, 50);              // Устанавливаем позицию курсора на дисплее
    u8g2.print("Дата    : Недоступна"); // Выводим текст на дисплей
  }

  // Проверяем, доступна ли информация о времени
  if (gps.time.isValid())
  {
    u8g2.setCursor(0, 60);                         // Позиционируем курсор на дисплее
    u8g2.print("Время   :");                       // Выводим текст на дисплей
    u8g2.print(gps.time.hour() < 10 ? "0" : "");   // Добавляем ведущий ноль, если час меньше  10
    u8g2.print(gps.time.hour());                   // Выводим часы
    u8g2.print(":");                               // Выводим разделитель
    u8g2.print(gps.time.minute() < 10 ? "0" : ""); // Добавляем ведущий ноль, если минута меньше  10
    u8g2.print(gps.time.minute());                 // Выводим минуты
    u8g2.print(":");                               // Выводим разделитель
    u8g2.print(gps.time.second() < 10 ? "0" : ""); // Добавляем ведущий ноль, если секунда меньше  10
    u8g2.print(gps.time.second());                 // Выводим секунды
    u8g2.print(" UTC");
  }
  else
  {
    u8g2.setCursor(0, 60);              // Позиционируем курсор н�� дисплее
    u8g2.print("Время   : Недоступно"); // Выводим текст на дисплее
  }
  u8g2.sendBuffer(); // Отправляем буфер на дисплей
}
*/

void loop()
{
    // Здесь начинается основной цикл программы
    while (Serial.available() > 0)
    {
        gps.encode(Serial.read());
    }

    if (gps.location.isUpdated() && gps.location.isValid())
    {
        gpsUpdated = true; // Устанавливаем флаг, что GPS обновлен
        // Включаем звуковой сигнал на пине  12, если он еще не был сгенерирован
        if (!signalPlayed)
        {
            tone(buzzerPin, 1000, 500); // Генерируем звуковой сигнал на частоте  1000 Гц в течение  500 мс
            signalPlayed = true;        // Устанавливаем флаг, что сигнал был сгенерирован
        }

        // Обработка нажатия кнопки
        buttonPinRead();

        // Отображение информации на экране, которая доступна в данный момент
        switch (menuState)
        {
        case 0:
            displayTimeAndDate(); // Отображаем время и дату
            break;
        case 1:
            displayMainGPSData(); // Отображаем основные данные GPS
            break;
        case 2:
            displaySecondaryGPSData(); // Отображаем вторичные данные GPS
            break;
        }
    }

    else if (!gpsUpdated)
    {
        animateImage(); // Запускаем анимацию, если GPS еще не обновлен
    }

    // Если кнопка была нажата, переключаем меню, независимо от состояния данных GPS
    else if (buttonPressed)
    {
        menuState = (menuState + 1) % 3; // Переключаем состояние меню
        buttonPressed = false;           // Сбрасываем флаг, что кнопка отпущена
        // Отображаем информацию на экране, которая доступна в данный момент
        switch (menuState)
        {
        case 0:
            displayTimeAndDate(); // Отображаем время и дату
            break;
        case 1:
            displayMainGPSData(); // Отображаем основные данные GPS
            break;
        case 2:
            displaySecondaryGPSData(); // Отображаем вторичные данные GPS
            break;
        }
    }
    // Проверяем, нет ли сигнала потери связи
    if (millis() > 500000 && gps.charsProcessed() < 10)
    {
        // Сигнал потери связи
        if (!signalPlayed)
        {
            tone(buzzerPin, 100, 1000); // Генерируем звуковой сигнал на частоте   1000 Гц в течение   1000 мс
            signalPlayed = true;        // Устанавливаем флаг, что сигнал был сгенерирован
        }
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_10x20_t_cyrillic); // Устанавливаем шрифт для дисплея
        u8g2.setCursor(5, 15);
        u8g2.print("СПУТНИКИ GPS");
        u8g2.setCursor(10, 35);
        u8g2.print("НЕДОСТУПНЫ!");
        u8g2.setFont(u8g2_font_6x13_t_cyrillic); // Устанавливаем шрифт для дисплея
        u8g2.setCursor(30, 50);
        u8g2.print("Выйдите под");
        u8g2.setCursor(23, 60);
        u8g2.print("открытое небо!");
        u8g2.sendBuffer();
        // Задержка перед отображением сообщения, чтобы дать время на обработку данных GPS
        delay(5000);
    }
}