#include "properties_dialog_delegate.hpp"

PropertiesDialogItemDelegate::PropertiesDialogItemDelegate(QObject *parent):
    m_ui_prop_dialog(new Ui::propertiesDialog), m_key_value_model(new QStandardItemModel)
{
    
}

PropertiesDialogItemDelegate::~PropertiesDialogItemDelegate()
{
    delete m_ui_prop_dialog;
}

void PropertiesDialogItemDelegate::addKeyValueButtonClicked()
{
    ROS_INFO_STREAM("Add key value button clicked");

    QStandardItem *item = new QStandardItem();
    m_key_value_model->appendRow(item);
    m_ui_prop_dialog->keyValueTableView->setModel(m_key_value_model);
}

void PropertiesDialogItemDelegate::deleteKeyValueButtonClicked()
{
    ROS_INFO_STREAM("Delete key value button clicked");
    
    QModelIndex index = m_ui_prop_dialog->keyValueTableView->selectionModel()->currentIndex();
    m_key_value_model->removeRow(index.row());
    m_ui_prop_dialog->keyValueTableView->setModel(m_key_value_model);
}

QWidget *PropertiesDialogItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    QDialog *d = new QDialog(parent);

    m_ui_prop_dialog->setupUi(d);
    d->setWindowTitle("Properties");
    d->setModal(true);
    d->setFocus(Qt::FocusReason::ActiveWindowFocusReason);

    m_key_value_model->clear();
    m_key_value_model->setColumnCount(2);
    m_key_value_model->setHorizontalHeaderLabels({"Key", "Value"});

    connect(m_ui_prop_dialog->addKeyValuePushButton, &QPushButton::clicked, this, &PropertiesDialogItemDelegate::addKeyValueButtonClicked);
    connect(m_ui_prop_dialog->deleteKeyValuePushButton, &QPushButton::clicked, this, &PropertiesDialogItemDelegate::deleteKeyValueButtonClicked);

    return d;
}

void PropertiesDialogItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    m_key_value_model->blockSignals(true);

    QStandardItem *item = nullptr;

    for(auto kv_it = m_key_value_qmap.begin(); kv_it != m_key_value_qmap.end(); ++kv_it)
    {
        item = new QStandardItem(kv_it.key());
        m_key_value_model->appendRow(item);
        m_key_value_model->setItem(m_key_value_model->rowCount()-1, 1, new QStandardItem(kv_it.value().toString()));
    }

    m_ui_prop_dialog->keyValueTableView->setModel(m_key_value_model);

    m_key_value_model->blockSignals(false);
}
    
void PropertiesDialogItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
    QDialog *d = qobject_cast<QDialog*>(editor);

    Q_ASSERT(d);

    if (!d || d->result() == QDialog::Rejected)
    {
        model->setData(index, m_key_value_qmap, Qt::DisplayRole);
        return;
    }

    ROS_INFO_STREAM("Properties key value pair");
    ROS_INFO_STREAM("Number of property key value pair(s): "<<m_key_value_model->rowCount());

    QMap<QString, QVariant> key_value_map;

    for(int i = 0; i < m_key_value_model->rowCount(); ++i)
    {
        if(m_key_value_model->item(i, 0)->text().size() == 0) 
        {
            m_key_value_model->removeRow(i);
            continue;
        }

        key_value_map.insert(m_key_value_model->item(i, 0)->text(), m_key_value_model->item(i, 1)->text());
        std::string key = m_key_value_model->item(i, 0)->text().toStdString();
        std::string value = m_key_value_model->item(i, 1)->text().toStdString();
        ROS_INFO_STREAM("Keys: "<<key<<" --- Value: "<<value);
    }

    m_key_value_qmap = key_value_map;

    model->setData(index, m_key_value_qmap, Qt::DisplayRole);
}

void PropertiesDialogItemDelegate::addKeyValueProperty(const std::string &key, const std::string &value)
{
    m_key_value_qmap.insert(QString::fromStdString(key), QString::fromStdString(value));
}
