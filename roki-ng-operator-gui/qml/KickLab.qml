import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ColumnLayout {
    id: lab
    property var trial: backend.calibration.result || ({})
    property var measurement: trial.kick_lab || ({})
    property var observation: backend.kickPreview || ({})
    function options() {
        return {leg: foot.currentIndex === 0 ? "right" : "left",
                strength: power.currentIndex === 0 ? "weak" : "strong",
                ball_forward_mm: front.value, ball_side_mm: side.value}
    }
    function place() { backend.previewKick(options()) }
    Label { text: "Поставьте мяч на схеме или введите координаты. Отпустите мяч, чтобы увидеть кадр камеры."; wrapMode: Text.Wrap; Layout.fillWidth: true }
    RowLayout {
        ComboBox { id: foot; objectName: "labFoot"; model: ["Правая стопа", "Левая стопа"]; onActivated: lab.place() }
        ComboBox { id: power; objectName: "labPower"; model: ["Слабый удар", "Сильный удар"]; onActivated: lab.place() }
    }
    Flow {
        Layout.fillWidth: true; spacing: 4
        Label { text: "Подъём стопы при сильном ударе, мм. В профиле: " + (backend.calibrationTilts["kick.strike_lift_mm"] ?? "—") }
        SpinBox { id: lift; objectName: "labLift"; from: 32; to: 70; value: backend.calibrationTilts["kick.strike_lift_mm"] ?? 55; editable: true }
        Button { objectName: "labSaveLift"; text: "Записать подъём"; onClicked: backend.saveStrikeLift(lift.value) }
    }
    Rectangle {
        id: map; objectName: "labField"; Layout.fillWidth: true; implicitHeight: 180; color: "#356348"
        property real sign: foot.currentIndex === 0 ? 1 : -1
        Rectangle { width: 1; height: parent.height; x: parent.width / 2; color: "white" }
        Rectangle { width: parent.width; height: 1; y: 3 * parent.height / 4; color: "white" }
        Rectangle {
            width: 14; height: 14; radius: 7; color: "#ff9300"
            x: map.width * (0.5 + map.sign * side.value / 400) - 7
            y: map.height * (.75 - front.value / 400) - 7
        }
        MouseArea {
            anchors.fill: parent
            function moveBall(mouse) {
                side.value = Math.round(map.sign * 400 * (mouse.x / width - .5))
                front.value = Math.round(400 * (.75 - mouse.y / height))
            }
            onPressed: mouse => moveBall(mouse)
            onPositionChanged: mouse => { if (pressed) moveBall(mouse) }
            onReleased: lab.place()
        }
    }
    Flow {
        Layout.fillWidth: true; spacing: 4
        Label { text: "Вперёд от центра робота, мм" }
        SpinBox { id: front; objectName: "labFront"; from: -100; to: 300; value: 100; editable: true }
        Label { text: "От оси к выбранной ноге, мм" }
        SpinBox { id: side; objectName: "labSide"; from: -200; to: 200; value: 52; editable: true }
        Button { objectName: "labPlace"; text: "Поставить мяч"; onClicked: lab.place() }
        Button { objectName: "labKick"; text: "Проверить удар"; onClicked: backend.startCalibrationTest("kick_test", lab.options()) }
    }
    Flow {
        Layout.fillWidth: true; spacing: 4
        Label { text: "Сдвиг, мм" }
        SpinBox { id: increment; from: 1; to: 20; value: 5; editable: true }
        Repeater {
            model: ["Вперёд", "Назад", "Влево", "Вправо"]
            delegate: Button {
                required property int index
                required property string modelData
                text: modelData
                onClicked: {
                    if (index < 2)
                        front.value += (index === 0 ? 1 : -1) * increment.value
                    else
                        side.value += (index === 2 ? -1 : 1) * map.sign * increment.value
                    lab.place()
                }
            }
        }
    }
    Image {
        Layout.fillWidth: true; Layout.preferredHeight: source.toString() ? 240 : 0
        fillMode: Image.PreserveAspectFit
        source: lab.observation.image || ""
    }
    Label {
        Layout.fillWidth: true; wrapMode: Text.Wrap
        text: lab.observation.visible ? "Камера: центр мяча (" + lab.observation.u + ", " + lab.observation.v
              + ") пикс. Дальность: " + (lab.observation.range_mm ?? "—") + " мм."
              : "Камера: подтверждённого наблюдения мяча нет."
    }
    Label {
        Layout.fillWidth: true; wrapMode: Text.Wrap
        text: lab.observation.target
              ? "Ближайшая проверенная точка мяча: вперёд " + lab.observation.target.front_mm
                + " мм, к выбранной стопе " + lab.observation.target.side_mm
                + " мм. Восемь соседних точек с шагом 5 мм также дали устойчивый удар."
              : "Для рекомендации нужны успешные пробы в точке и вокруг неё с шагом 5 мм."
    }
    Label {
        objectName: "labOutcome"; visible: !!lab.measurement.camera; Layout.fillWidth: true; wrapMode: Text.Wrap
        text: "Последний удар (" + (lab.trial.args?.leg === "left" ? "левая" : "правая") + ", "
              + (lab.trial.args?.strength === "weak" ? "слабый" : "сильный") + "): "
              + (lab.trial.ball_contact ? "контакт" : "промах") + ", мяч вперёд " + lab.trial.ball_dx_mm
              + " мм, " + (lab.trial.fallen || !lab.trial.upright ? "потеря стойки" : "стойка сохранена")
              + ". Пик вертикальной нагрузки от мяча на стопу: " + lab.trial.ball_support_n + " Н. Зачтено наблюдений: " + lab.measurement.count
              + (lab.measurement.count ? ". Крайние значения отдельных удачных проб, мм: дальность " + lab.measurement.bounds.range_mm.join(" … ")
              + ", вперёд " + lab.measurement.bounds.x_mm.join(" … ") + ", влево " + lab.measurement.bounds.y_mm.join(" … ") : "")
    }
}
