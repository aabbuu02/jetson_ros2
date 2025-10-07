#pragma once

#include <vector>

#include <QComboBox>
#include <QStyledItemDelegate>

class ComboBoxItemDelegate : public QStyledItemDelegate
{
    Q_OBJECT
    
    public:
    
    ComboBoxItemDelegate(QObject *parent = nullptr);
    ~ComboBoxItemDelegate();

    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
    void setEditorData(QWidget *editor, const QModelIndex &index) const override;
    void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;

    void addItemsToComboBox(std::vector<std::string> item_names);
    void setCurrentItemInComboBox(int item_pair_index);

    private:
    
    int m_current_item_idx = 0;
    std::vector<std::string> m_item_names;
};
