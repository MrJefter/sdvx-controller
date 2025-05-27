# DIY SDVX Controller

<img src="docs/images/SDVX%20Controller%20-%20Assembly%20Render.png" alt="SDVX Controller Assembly Render" width="600"/>

Привет! Это проект DIY контроллера для Sound Voltex и подобных ритм-игр. Он построен на базе микроконтроллера STM32 (Black Pill), с кастомной прошивкой, печатной платой и корпусом, напечатанным на 3D-принтере.

## Особенности проекта

*   **Микроконтроллер** STM32F401 (на плате Black Pill v3.0).
*   **Кнопки** 7 основных (BT-A, B, C, D, START) + 2 FX-кнопки (FX-L, FX-R) на CherryMX переключателях (например, Outemu Red).
*   **Энкодеры** 2 поворотных LPD3806 (VOL-L, VOL-R).
*   **Адресная RGB-подсветка** на светодиодах WS2812B (опционально, лента на PB0).
*   **Интерфейс** USB HID (контроллер определяется как стандартное игровое устройство).
*   **Полностью 3D-печатный корпус**.
*   **Печатная плата** для однослойного травления в домашних условиях, нет нужды заказывать плату.

## Демонстрация работы

<img src="docs/images/MVP.gif" alt="MVP Link" width="600"/>

<img src="docs/images/Full%20Assembly%20See-Through.jpg" alt="Full Assembly See-Through" width="600"/>

## Аппаратная часть

Все файлы, относящиеся к аппаратной части, находятся в папке [`hardware/`](./hardware/).

### Печатная плата
Проект печатной платы разработан в EasyEDA. Исходные файлы, Gerber-файлы для заказа и спецификация (BOM) находятся в `hardware/pcb/`.

| Схематик | Разводка платы |
|---|---|
| <img src="docs/images/Schematic.png" alt="Schematic" width="400"/> | <img src="docs/images/PCB.png" alt="PCB Layout" width="400"/> |

**Фотографии сборки PCB:**
| Лицевая сторона PCB | Оборотная сторона PCB |
|---|---|
| <img src="docs/images/PCB%20Front.jpg" alt="PCB Front" width="400"/> | <img src="docs/images/PCB%20Bottom.jpg" alt="PCB Bottom" width="400"/> |

### Корпус
3D-модели корпуса и его компонентов разработаны в КОМПАС-3D. Исходные файлы CAD, экспортированные STEP-файлы и STL-файлы для печати находятся в `hardware/enclosure/` и `hardware/stl_for_printing/`.

3D-модели используемых компонентов (BlackPill, энкодеры, переключатели) находятся в `hardware/enclosure/component_models_3d/`.

### Процесс сборки (фото)

| Подключение энкодеров к передней панели | Установка светодиодной ленты |
|---|---|
| <img src="docs/images/Front%20panel%20PCB%20and%20encoders%20attached.jpg" alt="Front panel PCB and encoders attached" width="400"/> | <img src="docs/images/LED%20Strip%20Installation.jpg" alt="LED Strip Installation" width="400"/> |

**Доработка BlackPill:**

При желании можно выполнить одну небольшую аппаратную модификацию для BlackPill, добавив керамический конденсатор на 150 нФ (как на иллюстрации), чтобы можно было уходить в DFU по зажатию START на 10 секунд.

<img src="docs/images/BlackPill%20Simple%20DFU%20Entrance%20Lifehack.jpg" alt="BlackPill DFU Lifehack" width="400"/>

## Программное обеспечение (Прошивка)

Прошивка для микроконтроллера STM32 разработана с использованием STM32CubeIDE. Исходный код находится в папке [`firmware/`](./firmware/).

Ключевые функции прошивки:
*   Опрос состояния кнопок и энкодеров.
*   Управление RGB-подсветкой, включая визуальные эффекты.
*   Передача данных на ПК по USB HID протоколу.

## Сборка и настройка

[ЗДЕСЬ БУДЕТ ПОДРОБНОЕ РУКОВОДСТВО ПО СБОРКЕ, СПИСОК КОМПОНЕНТОВ, ИНСТРУКЦИИ ПО ПРОШИВКЕ МК И НАСТРОЙКЕ]

Примерный порядок действий:
1.  **Напечатайте корпус**, используя STL-файлы из папки `hardware/stl_for_printing/`.
2.  **Закажите плату** по Gerber-файлам из `hardware/pcb/gerbers/` или **вытравите самостоятельно** по проекту из `hardware/pcb/easyeda_project/`.
3.  **Соберите плату**, припаяйте кнопки, энкодеры, светодиодные ленты.
4.  **Соберите корпус.**
5.  **Прошейте микроконтроллер** - скомпилируйте проект из `firmware/` (или воспользуйтесь готовым бинарником из релизов) и прошейте Black Pill (например, через DFU режим или с помощью ST-Link).
6.  **Вы великолепны!**

Более подробные инструкции по сборке и настройке будут добавлены в папку `docs/` позже (когда нибудь).

## Лицензия

Этот проект распространяется под лицензией **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)**.
