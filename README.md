# Roki FPV и симулятор Roki-2

Roki FPV — пульт оператора для камеры, параметров, игровых режимов и калибровки. Он подключается к голове Roki с программой roki-ng или к локальному симулятору MuJoCo. В репозитории лежат исходники пульта, программы робота, модели поля и измеренный учебный профиль.

**[Скачать выпуск для macOS и Windows](https://github.com/dimaystinov/Roki/releases/tag/fpv-calibration-2026.11.26)** · **[Статья о 20 калибровках](docs/calibration/Инструкция-по-калибровке.md)** · **[Видео MP4 с русской озвучкой, 8 мин 7 с](https://github.com/dimaystinov/Roki/releases/download/fpv-calibration-2026.11.26/Roki-Calibration-RU.mp4)**

## Запустить готовый пульт

| Платформа | Установка и запуск |
|---|---|
| macOS 26, Apple Silicon | Распакуйте [архив macOS arm64](https://github.com/dimaystinov/Roki/releases/download/fpv-calibration-2026.11.26/FPV-Roki-macOS-arm64.zip), перенесите **FPV Roki.app** в «Программы» и откройте. Сборка имеет локальную подпись. |
| Windows x64 | Распакуйте **весь** [архив Windows x64](https://github.com/dimaystinov/Roki/releases/download/fpv-calibration-2026.11.26/FPV-Roki-windows-x64.zip). Откройте **FPV Roki/FPV Roki.exe** либо запустите приложенный **Install-FPV-Roki.ps1** для установки в профиль пользователя. |

Во вкладке **Подключение** укажите адрес головы. Для прямого Ethernet в этой конфигурации используется `172.30.0.1`, порт `8093`. Нажмите **Подключить**. Для запуска остановленной камеры и изменения параметров нажмите **Получить управление**. Откройте **Стримы → Запросить видеовыходы → Камера → Смотреть**. Для симулятора укажите адрес машины, где работает MuJoCo, и порт `8094`. При запуске на том же компьютере адрес — `127.0.0.1`.

## Запустить учебное поле на Mac

Скачиваемый архив приложения содержит пульт. Физическая модель и программа симулятора запускаются из исходников. Команды ниже выполняются на Apple Silicon с Homebrew и инструментами разработчика Xcode. Сборка использует PySide6 той же версии, что и Qt из Homebrew.

```bash
git clone https://github.com/dimaystinov/Roki.git
cd Roki/roki-ng-operator-gui
brew install python@3.11 qt cmake ninja pkg-config
bash scripts/build_macos.sh
bash scripts/install_gstreamer_macos.sh
.deps/macos/venv/bin/python -m pip install 'mujoco>=3.3,<4' opencv-python
cd ..
bash 'Запустить Roki2 симулятор и FPV.command'
```

Для повторения измеренных опытов после установки откройте [«Открыть измеренный профиль»](docs/calibration/Открыть%20измеренный%20профиль.command). Обе команды запускают одну пару окон. В FPV выберите **SIM-ROKI2 → Получить управление → Ручной режим**.

## Откалибровать и сохранить

1. Выберите **Тесты → Калибровка и измерения**. Матч завершится. Симулятор установит робота на центральную отметку.
2. Выберите **На спине → Проверить вставание**. После финиша в стойке выполните **Rotation test**. Сверьте четыре коэффициента, нажмите **Записать в профиль симулятора** и дождитесь строки **Прочитано из профиля**.
3. Запустите **Short run**, затем **Long run**. Смотрите X, Y, курс и конечную стойку в MuJoCo и в блоке **Измерение** в FPV. Продвижение и сумму команд вводите в соответствующие поля целыми миллиметрами.
4. Для удара выберите ногу, силу и положение мяча. Сравните контакт, путь мяча и стойку после возврата. Положение мяча задаётся относительно центра робота. В учебном профиле высота подъёма стопы при сильном ударе составляет 45 мм.
5. Для игры переключитесь в **Футбол**, откройте **Игра**, выберите `simple_football`, стартовую позицию и нажмите **Запустить игру**.

Обычный симулятор хранит настройки в `roki-ng-operator-gui/sim/.local-state/parameters.json`. Измеренный пример лежит в [training-profile](docs/calibration/training-profile). Для собственного профиля задайте абсолютный путь каталога перед запуском. Повторный запуск с тем же каталогом загрузит сохранённые значения.

```bash
ROKI_SIM_STATE_DIR="$PWD/my-training-profile" bash 'Запустить Roki2 симулятор и FPV.command'
```

Для переноса результатов скопируйте каталог профиля целиком. Он содержит параметры и модель робота, а после измерения зоны удара — журнал и снимки в `kick-lab/`. На физическом роботе измерения проводятся отдельно. В видео слева показано движение, справа — нажатия в FPV. В статье приведены таймкоды всех испытаний. [Статья](docs/calibration/Инструкция-по-калибровке.md) · [Карта 1368 проб удара](docs/calibration/KICK-GRID.md).

## Штатное вставание со спины

Для физического Roki-2 команда `motion.get_up` при положении `back` выбирает оригинальный слот **Roki_2_Get_UP_Face_Up**. Файл содержит кадры сервоприводов и названия страниц из Roki_2_Soccer. Калибровочные измерения записываются в профиль. MuJoCo для воспроизведения устойчивого подъёма применяет отдельную последовательность `sim/recovery/back.json`. [Оригинальный слот робота](roki-ng/roki_ng/body/motion/backends/roki2/slots/Roki_2_Get_UP_Face_Up.json) · [Подъём в симуляторе](roki-ng-operator-gui/sim/recovery/back.json).

## Сборка пульта из исходников

**macOS.** После установки зависимостей выше выполните в каталоге `roki-ng-operator-gui` следующие команды. Приложение появится в `dist/FPV Roki.app`.

```bash
.deps/macos/venv/bin/python -m pip install 'pyinstaller>=6,<7'
bash packaging/build_macos.sh
'dist/FPV Roki.app/Contents/MacOS/FPV Roki' --self-test-video
```

**Windows x64.** Используйте Developer PowerShell Visual Studio 2022 с инструментами C++ x64 и Miniforge. В каталоге `roki-ng-operator-gui` выполните следующие команды. Упаковщик проверяет видео, QML и загрузку окна. Готовый ZIP находится в `dist`.

```powershell
conda env create -f packaging/windows-environment.yml
conda activate fpv-roki-build
python -m pip install "mujoco>=3.3,<4" opencv-python-headless pytest
./packaging/build_windows.ps1
```

Состояние сборок и журналы доступны в [GitHub Actions](https://github.com/dimaystinov/Roki/actions/workflows/fpv-windows.yml). Исходные проекты Алексея Ягубова — [roki-ng](https://github.com/aleksey-yagubov/roki-ng) и [roki-ng-operator-gui](https://github.com/aleksey-yagubov/roki-ng-operator-gui). [Состав и лицензии поставки](roki-ng-operator-gui/packaging/THIRD_PARTY.md).
