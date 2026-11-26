import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ColumnLayout {
    id: panel
    property var state: backend.calibration
    property var trial: state.result || ({})
    property var proposal: trial.proposed || trial.values || ({})
    property string keySignature: JSON.stringify(Object.keys(proposal))
    property var keys: JSON.parse(keySignature)
    property bool ready: controls.view.ready && !state.busy
    property bool measurements: ready && state.phase === "ready"
    property string phaseLabel: ({stand: "Проверьте стойку или выполните вставание.",
                                 rotation: "Выполните Rotation test.",
                                 save_rotation: "Проверьте и запишите четыре коэффициента поворота.",
                                 ready: "Выберите движение для измерения."})[state.phase] || ""
    function run(name, args) { backend.startCalibrationTest(name, args || {}) }
    function mm(value) { return Number.isFinite(value) ? Math.round(value) + " мм" : "—" }
    function angle(value) { return Number.isFinite(value) ? value.toFixed(1) + "°" : "—" }
    function signedAngle(value) { return Number.isFinite(value) ? (value > 0 ? "+" : "") + angle(value) : "—" }
    function save() {
        let values = {}
        for (let i = 0; i < entries.count; ++i)
            values[keys[i]] = entries.itemAt(i).value
        backend.saveCalibration(trial.job_id, values)
    }

    Label { text: panel.phaseLabel; font.bold: true; Layout.fillWidth: true; wrapMode: Text.Wrap }
    Label { text: "Исходная поза" }
    ComboBox { id: fixture; objectName: "calibrationFixture"; model: ["Стойка", "На животе", "На спине", "На левом боку", "На правом боку"] }
    Flow {
        Layout.fillWidth: true; spacing: 4
        Button {
            objectName: "calibrationStandButton"; text: "Проверить вставание"
            enabled: panel.ready && ["stand", "ready"].includes(panel.state.phase)
            onClicked: panel.run("get_up_test", {fixture: ["upright","front","back","left","right"][fixture.currentIndex]})
        }
        Button {
            objectName: "calibrationRotationButton"; text: "Rotation test"
            enabled: panel.ready && ["rotation", "save_rotation", "ready"].includes(panel.state.phase)
            onClicked: panel.run("rotation_test")
        }
        Button {
            objectName: "calibrationStopButton"; text: "Остановить тест"
            enabled: controls.view.owns && (panel.state.busy || controls.view.moving)
            onClicked: controls.stop(false)
        }
    }
    Flow {
        Layout.fillWidth: true; spacing: 4
        enabled: panel.measurements
        Button { objectName: "calibrationShortButton"; text: "Short run"; onClicked: panel.run("run_test", {mode: "short"}) }
        Button { objectName: "calibrationLongButton"; text: "Long run"; onClicked: panel.run("run_test", {mode: "long"}) }
        Button { objectName: "calibrationSpotButton"; text: "На месте"; onClicked: panel.run("run_test", {mode: "spot"}) }
        Button { objectName: "calibrationBackButton"; text: "Назад"; onClicked: panel.run("run_test", {mode: "backwards"}) }
        Button { objectName: "calibrationSideLeftButton"; text: "Шаг влево"; onClicked: panel.run("run_test", {mode: "side_left"}) }
        Button { objectName: "calibrationSideRightButton"; text: "Шаг вправо"; onClicked: panel.run("run_test", {mode: "side_right"}) }
    }
    Flow {
        Layout.fillWidth: true; spacing: 4
        enabled: panel.measurements
        Label { text: "Циклы" }
        SpinBox { id: cycles; from: 1; to: 100; value: 10; editable: true }
        Label { text: "Шаг, мм" }
        SpinBox { id: step; from: -64; to: 64; value: 24; editable: true }
        Label { text: "Вбок, мм" }
        SpinBox { id: side; from: 0; to: 20; editable: true }
        CheckBox { id: rightLeg; text: "Правая нога"; checked: true }
        Button { objectName: "calibrationCustomButton"; text: "Свой проход"; onClicked: panel.run("run_test", {mode: "custom", cycles: cycles.value, step_mm: step.value, side_mm: side.value, right_leg: rightLeg.checked}) }
    }
    Flow {
        Layout.fillWidth: true; spacing: 4
        enabled: panel.measurements
        ComboBox { id: jumpDir; objectName: "calibrationJumpDirection"; model: ["Вперёд", "Назад", "Влево", "Вправо", "На месте"] }
        SpinBox { id: jumps; objectName: "calibrationJumpCount"; from: 1; to: 100; value: 10; editable: true }
        Button { objectName: "calibrationJumpButton"; text: "Серия прыжков"; onClicked: panel.run("jump_test", {direction: ["forward","backward","left","right","on_spot"][jumpDir.currentIndex], count: jumps.value}) }
    }
    Flow {
        Layout.fillWidth: true; spacing: 4
        enabled: panel.measurements
        ComboBox { id: foot; objectName: "calibrationKickLeg"; model: ["Правая стопа", "Левая стопа"] }
        CheckBox { id: placeBall; text: "Задать положение мяча" }
        Label { visible: placeBall.checked; text: "Вперёд, мм" }
        SpinBox { id: ballForward; visible: placeBall.checked; from: -1500; to: 1500; value: 100; editable: true }
        Label { visible: placeBall.checked; text: "Кнаружи от стопы, мм" }
        SpinBox { id: ballSide; visible: placeBall.checked; from: -1500; to: 1500; value: 52; editable: true }
        Button { objectName: "calibrationKickButton"; text: "Удар по мячу"; onClicked: {
            const args = {leg: foot.currentIndex === 0 ? "right" : "left", mode: "regular"}
            if (placeBall.checked) { args.ball_forward_mm = ballForward.value; args.ball_side_mm = ballSide.value }
            panel.run("kick_test", args)
        } }
    }
    ColumnLayout {
        visible: !!panel.trial.job_id; Layout.fillWidth: true
        Label {
            text: "Измерение № " + (panel.trial.job_id || "") + " · "
                + (({completed: "завершено", cancelled: "отменено", failed: "ошибка"})[panel.trial.status] || panel.trial.status || "")
            font.bold: true
        }
        Label {
            visible: Number.isFinite(panel.trial.start_x_mm) && Number.isFinite(panel.trial.start_y_mm)
            text: "Отметка старта: X " + panel.mm(panel.trial.start_x_mm) + " · Y " + panel.mm(panel.trial.start_y_mm)
            Layout.fillWidth: true; wrapMode: Text.Wrap
        }
        Label {
            objectName: "calibrationMeasured"
            Layout.fillWidth: true; wrapMode: Text.Wrap
            text: "Вперёд " + panel.mm(panel.trial.dx_mm) + " · вбок " + panel.mm(panel.trial.dy_mm)
                + " · путь " + panel.mm(panel.trial.path_mm) + " · поворот " + panel.angle(panel.trial.yaw_deg)
        }
        Label {
            objectName: "calibrationPitch"
            Layout.fillWidth: true; wrapMode: Text.Wrap
            text: "Тангаж корпуса (+ вперёд): финиш " + panel.signedAngle(panel.trial.pitch_deg)
                + " · минимум " + panel.signedAngle(panel.trial.pitch_min_deg)
                + " · максимум " + panel.signedAngle(panel.trial.pitch_max_deg)
        }
        Label {
            text: (panel.trial.fallen ? "Зафиксировано падение. " : "")
                + (panel.trial.upright === true ? "Финиш в стойке." : panel.trial.upright === false ? "На финише вне стойки." : "")
            Layout.fillWidth: true; wrapMode: Text.Wrap
        }
        Label {
            visible: panel.trial.test === "run_test"
            text: panel.trial.odometry ? "Одометрия: вперёд " + panel.mm(panel.trial.odometry.dx_mm) + " · вбок " + panel.mm(panel.trial.odometry.dy_mm) : "Одометрия недоступна"
            Layout.fillWidth: true; wrapMode: Text.Wrap
        }
        Label {
            visible: panel.trial.test === "jump_test"
            text: "Завершено прыжков: " + (panel.trial.repeats ?? "—") + " · среднее вперёд / вбок: "
                + panel.mm(panel.trial.dx_mm / panel.trial.repeats) + " / " + panel.mm(panel.trial.dy_mm / panel.trial.repeats)
            Layout.fillWidth: true; wrapMode: Text.Wrap
        }
        Label {
            visible: panel.trial.test === "kick_test"
            text: (panel.trial.ball_contact ? "Контакт выбранной стопы" : "Промах") + " · мяч вперёд "
                + panel.mm(panel.trial.ball_dx_mm) + " · дальность " + panel.mm(panel.trial.ball_distance_mm) + " · отклонение " + panel.mm(panel.trial.ball_dy_mm)
            Layout.fillWidth: true; wrapMode: Text.Wrap
        }
        Label {
            visible: Number.isFinite(panel.trial.verification_error_deg)
            text: "Ошибка контрольного поворота: " + panel.signedAngle(panel.trial.verification_error_deg)
            Layout.fillWidth: true; wrapMode: Text.Wrap
        }
        SelectableLabel {
            text: panel.trial.error || ""
            visible: text !== ""
            Layout.fillWidth: true; wrapMode: Text.Wrap
        }
        Repeater {
            id: entries
            model: panel.keys
            delegate: ColumnLayout {
                id: entry
                required property string modelData
                property alias value: input.text
                readonly property var job: panel.trial.job_id
                function reset() { input.text = String(panel.proposal[modelData] ?? "") }
                Component.onCompleted: reset()
                onJobChanged: Qt.callLater(reset)
                Layout.fillWidth: true
                Label {
                    Layout.fillWidth: true; wrapMode: Text.WrapAnywhere
                    text: entry.modelData + (entry.modelData.endsWith("_mm") ? " (мм)" : entry.modelData.includes("yaw") ? " (рад/прыжок)" : entry.modelData.includes("rotation_yield") ? " (рад/цикл)" : "")
                }
                Label { text: "Было: " + ((panel.trial.previous || {})[entry.modelData] ?? "—") + " · предложено: " + (panel.proposal[entry.modelData] ?? "—") }
                RowLayout {
                    Label { text: "Записать" }
                    TextField { id: input; objectName: "calibrationValue_" + entry.modelData; selectByMouse: true; Layout.fillWidth: true }
                }
                Label {
                    property var stored: backend.calibrationReadback[entry.modelData]
                    visible: stored !== undefined
                    text: "Прочитано из профиля: " + (stored ?? "")
                }
            }
        }
        Flow {
            Layout.fillWidth: true; spacing: 4
            Button { objectName: "calibrationSaveButton"; text: "Записать в профиль симулятора"; enabled: panel.ready && entries.count > 0 && !panel.trial.saved; onClicked: panel.save() }
            Button { objectName: "calibrationRepeatButton"; text: "Повторить с отметки"; enabled: panel.ready; onClicked: backend.repeatCalibration() }
        }
    }
    CheckBox { id: labSwitch; text: "Калибровка зоны удара"; objectName: "kickLabSwitch"; enabled: panel.measurements }
    Loader {
        active: labSwitch.checked
        visible: active
        Layout.fillWidth: true
        enabled: panel.measurements
        source: active ? "KickLab.qml" : ""
    }
    Label { text: "Наклон корпуса"; font.bold: true }
    Label { text: "Компонента X вектора стопы, без единиц (−0,3 … +0,3). Плюс — вперёд."; Layout.fillWidth: true; wrapMode: Text.Wrap }
    Repeater {
        model: ["walk.body_tilt_forward", "kick.body_tilt", "kick.recovery_tilt"]
        delegate: ColumnLayout {
            required property string modelData
            property var stored: backend.calibrationTilts[modelData]
            Layout.fillWidth: true
            Label { text: modelData + " · в профиле: " + (stored ?? "—"); Layout.fillWidth: true; wrapMode: Text.WrapAnywhere }
            RowLayout {
                TextField { id: tilt; objectName: "calibrationTilt_" + modelData; text: String(stored ?? ""); selectByMouse: true; Layout.fillWidth: true }
                Button { objectName: "calibrationSaveTilt_" + modelData; text: "Записать"; enabled: panel.ready && stored !== undefined; onClicked: backend.saveCalibrationTilt(modelData, tilt.text) }
            }
        }
    }
}
