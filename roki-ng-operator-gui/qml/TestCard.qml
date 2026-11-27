import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

GroupBox {
    id: card
    property var schema: ({})
    property string testName
    property string raw
    property var values: ({})
    property var testJob: ({})
    readonly property bool running: ["accepted", "running", "saving"].indexOf(testJob.status) >= 0
    readonly property bool showProgress: running && typeof testJob.progress === "number"
                                        && typeof (testJob.test || {}).progress_max === "number"
                                        && testJob.test.progress_max > 0
    property var inputValues: ({})
    property var inputFields: backend.testMeasurements[String(testJob.job_id)] || ({})
    property var inputKeys: testJob.manual_parameters || []
    onInputKeysChanged: {
        if (inputKeys.length) backend.loadTestMeasurements(String(testJob.job_id), inputKeys)
    }
    function setMeasurement(key, value) {
        let copy = Object.assign({}, inputValues); copy[key] = value; inputValues = copy
    }
    function updateJob() {
        const job = controls.view.job
        if (job.operation === "test.start" && (job.test || {}).name === testName)
            {
                if (testJob.job_id !== job.job_id) inputValues = {}
                testJob = Object.assign({}, job)
            }
    }
    Component.onCompleted: updateJob()
    Connections {
        target: controls
        function onChanged() { card.updateJob() }
    }
    title: schema.title || testName
    function defaults() {
        let result = {}
        for (let key in (schema.parameters || {})) {
            let field = schema.parameters[key]
            result[key] = field.default ?? (field.choices ? field.choices[0] : "")
        }
        values = result
    }
    onSchemaChanged: defaults()
    function update(key, value) { let copy = Object.assign({}, values); copy[key] = value; values = copy }
    ColumnLayout {
        anchors.fill: parent
        Label { visible: !card.schema.parameters; text: "Описание запрашивается…" }
        Label { visible: card.schema.body_imu === true; text: "Использует IMU тела"; Layout.fillWidth: true; wrapMode: Text.Wrap }
        Repeater {
            model: Object.keys(card.schema.parameters || {})
            delegate: ColumnLayout {
                required property string modelData
                property var field: card.schema.parameters[modelData]
                visible: !field.when || card.values.mode === field.when
                Layout.fillWidth: true
                Label { text: modelData + (field.min !== undefined ? " [" + field.min + " … " + field.max + "]" : ""); Layout.fillWidth: true; wrapMode: Text.Wrap }
                ValueEditor { Layout.fillWidth: true; meta: field; initialValue: field.default ?? (field.choices ? field.choices[0] : ""); onEdited: value => card.update(modelData, value) }
            }
        }
        Label {
            Layout.fillWidth: true
            wrapMode: Text.Wrap
            text: (card.schema.unavailable_modes || {})[card.values.mode] || ""
            visible: text !== ""
        }
        Button {
            objectName: "startTest_" + card.testName
            text: "Запустить"
            enabled: controls.view.ready && !!card.schema.parameters && !(card.schema.unavailable_modes || {})[card.values.mode]
            onClicked: backend.startTest(card.testName, card.values)
        }
        ProgressBar {
            objectName: "testProgress_" + card.testName
            Layout.fillWidth: true
            visible: card.showProgress
            from: 0
            to: (card.testJob.test || {}).progress_max || 1
            value: card.testJob.progress || 0
        }
        Label {
            objectName: "testResult_" + card.testName
            Layout.fillWidth: true
            wrapMode: Text.Wrap
            visible: !!card.testJob.status
            text: card.running
                  ? (card.testJob.stage || "Запускается…") + (card.showProgress ? " · " + card.testJob.progress + "/" + card.testJob.test.progress_max : "")
                  : card.testJob.status === "completed" ? "Успешно"
                  : card.testJob.status === "cancelled" ? "Отменён" + (card.testJob.reason ? ": " + card.testJob.reason : "")
                  : card.testJob.status === "failed" ? "Ошибка: " + (card.testJob.reason || "Причина не указана")
                  : card.testJob.status || ""
        }
        ColumnLayout {
            visible: card.testJob.status === "completed" && card.inputKeys.length > 0
            Layout.fillWidth: true
            Label { text: "Введите измеренный результат"; font.bold: true }
            Repeater {
                model: card.inputKeys
                delegate: ColumnLayout {
                    required property string modelData
                    Layout.fillWidth: true
                    Label {
                        text: modelData + " · " + (card.inputFields[modelData]?.description || "")
                        Layout.fillWidth: true; wrapMode: Text.Wrap
                    }
                    TextField {
                        objectName: "testMeasurement_" + modelData
                        Layout.fillWidth: true; selectByMouse: true
                        placeholderText: card.inputFields[modelData]?.type === "int" ? "Целое число" : "Измеренное значение"
                        onTextEdited: card.setMeasurement(modelData, text)
                    }
                }
            }
            Button {
                objectName: "saveTestMeasurement_" + card.testName
                text: "Записать измерения"
                enabled: controls.view.ready && card.inputKeys.every(key => card.inputValues[key] !== undefined && card.inputValues[key] !== "")
                onClicked: backend.measureTest(String(card.testJob.job_id), card.inputValues)
            }
        }
        SelectableLabel {
            Layout.fillWidth: true; wrapMode: Text.Wrap
            property var saved: backend.testMeasurementSaved[String(card.testJob.job_id)]
            visible: !!saved
            text: "Сохранено и прочитано из профиля: " + JSON.stringify(saved || {})
        }
        RawDetails { text: card.testJob.measurement ? JSON.stringify(card.testJob.measurement, null, 2) : ""; visible: !!card.testJob.measurement }
        RawDetails { text: card.raw }
    }
}
