import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ScrollView {
    id: scroll
    property var kddockwidgets_min_size: Qt.size(300, 220)
    contentWidth: availableWidth
    clip: true
    ColumnLayout {
        width: scroll.availableWidth
        spacing: 8
        Flow {
            Layout.fillWidth: true
            spacing: 6
            Button {
                objectName: "statusButton"; text: "Обновить"
                enabled: backend.view.connected
                onClicked: { backend.requestStatus(); backend.requestCapabilities() }
            }
        }
        Label { text: "Робот: " + backend.view.robot; Layout.fillWidth: true; wrapMode: Text.Wrap }
        Label { text: "Режим: " + backend.view.mode + (backend.view.connected ? "" : " (неактуально)"); Layout.fillWidth: true; wrapMode: Text.Wrap }
        Label { text: "Владелец управления: " + backend.view.owner; Layout.fillWidth: true; wrapMode: Text.Wrap }
        Label {
            objectName: "batteryPanelStatus"
            text: "Аккумулятор: " + dataSources.power.text
            Layout.fillWidth: true
            wrapMode: Text.Wrap
            HoverHandler { id: powerHover }
            ToolTip.visible: powerHover.hovered
            ToolTip.text: dataSources.power.details
        }
        Label { text: "Возраст снимка: " + backend.view.statusAge + " с" }
        Repeater {
            model: workersModel
            delegate: Label {
                required property var entry
                Layout.fillWidth: true
                wrapMode: Text.Wrap
                text: entry.name + ": " + entry.state + (entry.alive ? "" : entry.state === "stopped" ? " / не запущен" : " / процесс недоступен")
            }
        }
        RawDetails {
            objectName: "statusDetails"
            Layout.fillWidth: true
            text: backend.view.statusText
        }
        Label { text: "Связь и клиент"; font.bold: true }
        SelectableLabel { text: "RTT ответов: " + backend.view.rtt + " мс" }
        SelectableLabel { text: "Пропуски локальной очереди: " + backend.view.gaps }
        SelectableLabel { text: "Некорректные пакеты: " + backend.view.invalid }
        SelectableLabel { text: "Видеоприёмников: " + streams.view.active.length + " · просмотров: " + videoViews.entries.length; wrapMode: Text.Wrap; Layout.fillWidth: true }
        SelectableLabel { text: streams.view.error; visible: text.length > 0; wrapMode: Text.Wrap; Layout.fillWidth: true }
        SelectableLabel { text: backend.view.notice || "Ошибок команд нет"; Layout.fillWidth: true; wrapMode: Text.WrapAnywhere }
        Label { text: "Поддерживаемые функции"; font.bold: true }
        Label { text: "Режимы: " + (backend.view.capabilities.modes || []).join(", "); Layout.fillWidth: true; wrapMode: Text.Wrap }
        Label { text: "Движения: " + (backend.view.capabilities.body || []).join(", "); Layout.fillWidth: true; wrapMode: Text.Wrap }
        Label { text: "Источники данных: " + (backend.view.capabilities.data_topics || []).join(", "); Layout.fillWidth: true; wrapMode: Text.Wrap }
        RawDetails { text: backend.view.capabilitiesText }
    }
}
