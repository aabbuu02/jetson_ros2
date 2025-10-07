#include "combo_box_delegate.hpp"
#include <QComboBox>

ComboBoxItemDelegate::ComboBoxItemDelegate(QObject *parent)
    : QStyledItemDelegate(parent)
{

}


ComboBoxItemDelegate::~ComboBoxItemDelegate()
{

}

void ComboBoxItemDelegate::addItemsToComboBox(std::vector<std::string> item_names)
{
    m_item_names = item_names;
}

void ComboBoxItemDelegate::setCurrentItemInComboBox(int item_index)
{
    m_current_item_idx = item_index;
}


QWidget *ComboBoxItemDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    // Create the combobox and populate it
    QComboBox *cb = new QComboBox(parent);
    for(int i = 0; i < m_item_names.size(); ++i)
    {
        cb->addItem(QString::fromStdString(m_item_names.at(i)), {i});
    }
    cb->setCurrentIndex(m_current_item_idx);
    return cb;
}


void ComboBoxItemDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    QComboBox *cb = qobject_cast<QComboBox *>(editor);
    
    Q_ASSERT(cb);
    
    //Get the index of the text in the combobox that matches the current value of the item
    const QString current_text = index.data(Qt::EditRole).toString();
    const int cb_index = cb->findText(current_text);
    
    //If it is valid, adjust the combobox
    if (cb_index >= 0) cb->setCurrentIndex(cb_index);
}


void ComboBoxItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
    QComboBox *cb = qobject_cast<QComboBox *>(editor);
    Q_ASSERT(cb);
    model->setData(index, cb->currentText(), Qt::EditRole);
}
  