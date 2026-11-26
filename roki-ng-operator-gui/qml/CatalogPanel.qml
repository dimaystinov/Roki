import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ColumnLayout {
    id: root
    required property string kind
    property var kddockwidgets_min_size: Qt.size(320, 260)
    readonly property bool simulator: root.kind === "tests" && backend.view.connected && backend.view.robot === "SIM-ROKI2"
    readonly property bool measuring: simulator && backend.calibration.enabled === true
    Flow {
        visible: root.simulator
        Layout.fillWidth: true
        spacing: 4
        Button {
            objectName: root.kind === "tests" ? "footballModeButton" : ""
            text: "Футбол"
            highlighted: backend.calibration.enabled === false
            enabled: controls.view.owns && !controls.view.pending && root.measuring && !backend.calibration.busy
            onClicked: backend.setCalibrationMode(false)
        }
        Button {
            objectName: root.kind === "tests" ? "calibrationModeButton" : ""
            text: "Калибровка и измерения"
            highlighted: root.measuring
            enabled: controls.view.owns && !controls.view.pending && !root.measuring
            onClicked: backend.setCalibrationMode(true)
        }
    }
    Connections {
        target: slotsModel
        function onModelReset() { selectedSlot.text = ""; catalog.currentIndex = -1 }
    }
    Label { objectName: root.kind + "BlockedReason"; text: controls.view.blockedReason; visible: text !== "" && !root.measuring; Layout.fillWidth: true; wrapMode: Text.Wrap }
    Button { objectName: root.kind + "ManualButton"; text: "Включить ручной режим"; visible: !root.measuring && controls.view.owns && !controls.view.manual; enabled: controls.view.canEnterManual; onClicked: controls.enterManual() }
    Button {
        objectName: root.kind + "Button"
        visible: !root.measuring
        text: root.kind === "slots" ? "Запросить список слотов" : "Запросить тесты и формы"
        enabled: backend.view.connected && !backend.view.catalogsBusy
        onClicked: backend.requestCatalog(root.kind)
    }
    ListView {
        id: catalog
        objectName: root.kind + "List"
        visible: root.kind === "slots"
        Layout.fillWidth: true
        Layout.fillHeight: true
        Layout.minimumHeight: 80
        clip: true
        model: root.kind === "slots" ? slotsModel : null
        ScrollBar.vertical: ScrollBar {}
        delegate: ItemDelegate {
            required property var entry
            required property int index
            width: ListView.view.width
            text: entry.name
            highlighted: ListView.isCurrentItem
            onClicked: { catalog.currentIndex = index; selectedSlot.text = entry.name }
        }
    }
    RowLayout {
        visible: root.kind === "slots"
        Layout.fillWidth: true
        Label { id: selectedSlot; Layout.fillWidth: true; elide: Text.ElideRight; text: "" }
        Button { objectName: root.kind === "slots" ? "startSlotButton" : ""; text: "Запустить слот"; enabled: controls.view.ready && selectedSlot.text !== ""; onClicked: controls.slot(selectedSlot.text, 1.0) }
    }
    Label { visible: root.kind === "slots" && selectedSlot.text === ""; text: "Выберите слот в списке."; Layout.fillWidth: true; wrapMode: Text.Wrap }
    ScrollView {
        id: testScroll
        objectName: root.kind === "tests" ? "testsScroll" : ""
        visible: root.kind === "tests"
        Layout.fillWidth: true
        Layout.fillHeight: true
        clip: true
        contentWidth: availableWidth
        ColumnLayout {
            width: testScroll.availableWidth
            Loader {
                Layout.fillWidth: true
                active: root.measuring
                source: "CalibrationPanel.qml"
            }
            Repeater {
                objectName: root.kind === "tests" ? "testCards" : ""
                model: root.kind === "tests" && !root.measuring ? testsModel : null
                delegate: TestCard { required property var entry; schema: entry.schema || ({}); testName: entry.name; raw: entry.raw || ""; Layout.fillWidth: true }
            }
        }
    }
    Flow {
        visible: !root.measuring
        Layout.fillWidth: true
        spacing: 4
        Button { text: "Завершить цикл"; enabled: controls.view.owns; onClicked: controls.stop(false) }
        Button { text: "Сброс очереди"; enabled: controls.view.owns; onClicked: controls.stop(true) }
    }
    Label { text: controls.view.jobOperation + " · " + controls.view.jobStatus + " " + controls.view.jobProgress; Layout.fillWidth: true; wrapMode: Text.Wrap }
    SelectableLabel { text: controls.view.error || controls.view.jobReason; visible: text !== ""; Layout.fillWidth: true; wrapMode: Text.Wrap }
}
