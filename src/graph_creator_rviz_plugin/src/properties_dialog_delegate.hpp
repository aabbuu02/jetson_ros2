#pragma once

#include <memory>

#include <QMap>
#include <QVariant>
#include <QItemDelegate>
#include <QStandardItem>
#include <QStandardItemModel>

#include <ros/ros.h>

#include "properties_ui.hpp"

class PropertiesDialogItemDelegate: public QItemDelegate
{
    Q_OBJECT
    public:

    PropertiesDialogItemDelegate(QObject *parent=nullptr);
    ~PropertiesDialogItemDelegate();

    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
    void setEditorData(QWidget *editor, const QModelIndex &index) const override;
    void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;

    private Q_SLOT:

    void addKeyValueButtonClicked();
    void deleteKeyValueButtonClicked();

    public:

    void addKeyValueProperty(const std::string &key, const std::string &value);

    private:

    //TODO: Better way to track the key value map needs to be implemented (Probably using QStandardItemModel itself)
    mutable QMap<QString, QVariant> m_key_value_qmap;

    Ui::propertiesDialog *m_ui_prop_dialog;
    QStandardItemModel *m_key_value_model;
};