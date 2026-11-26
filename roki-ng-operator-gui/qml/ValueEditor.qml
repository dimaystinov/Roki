import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

RowLayout {
    id: root
    property var meta: ({})
    property var initialValue: undefined
    property bool commitOnFinish: false
    property bool preserveDraft: false
    property bool dirty: false
    property string choicesSignature: JSON.stringify(meta.choices || [])
    property var choiceValues: JSON.parse(choicesSignature)
    readonly property var value: meta.type === "object" ? objectEditor.value : meta.choices ? choice.currentText : meta.type === "bool" ? boolInput.checked : number.text
    signal edited(var value)
    function reset() {
        resetValue(initialValue)
    }
    function resetValue(v) {
        dirty = false
        if (meta.choices)
            choice.currentIndex = Math.max(0, meta.choices.indexOf(v))
        else if (meta.type === "bool")
            boolInput.checked = v === true
        else if (meta.type !== "object")
            number.text = v === undefined ? "" : String(v)
    }
    onInitialValueChanged: { if (!(preserveDraft && dirty) && (!commitOnFinish || !number.activeFocus)) reset() }
    // params.get may precede params.describe; reset after the choice model binds.
    onMetaChanged: { if (meta.choices) Qt.callLater(function() { if (!(preserveDraft && dirty)) reset() }) }
    Component.onCompleted: reset()
    ComboBox {
        id: choice
        objectName: "valueChoice"
        visible: !!root.meta.choices
        Layout.fillWidth: true
        model: root.choiceValues
        onActivated: { root.dirty = true; root.edited(currentText) }
    }
    CheckBox {
        id: boolInput
        visible: !root.meta.choices && root.meta.type === "bool"
        text: checked ? "Да" : "Нет"
        onToggled: { root.dirty = true; root.edited(checked) }
    }
    TextField {
        id: number
        objectName: "valueInput"
        visible: !root.meta.choices && root.meta.type !== "bool" && root.meta.type !== "object"
        Layout.fillWidth: true
        selectByMouse: true
        placeholderText: root.meta.type === "int" ? "Целое число" : "Значение"
        onTextEdited: { root.dirty = true; if (!root.commitOnFinish) root.edited(text) }
        onEditingFinished: { if (root.commitOnFinish) root.edited(text) }
    }
    ObjectEditor {
        id: objectEditor
        visible: root.meta.type === "object"
        fields: root.meta.fields || ({})
        initialValue: root.meta.type === "object" ? (root.initialValue || {}) : ({})
        Layout.fillWidth: true
        onEdited: function(value) { root.edited(value) }
    }
}
