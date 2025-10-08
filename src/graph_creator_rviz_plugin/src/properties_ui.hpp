/********************************************************************************
** Form generated from reading UI file 'properties.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef PROPERTIES_UI_H
#define PROPERTIES_UI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableView>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_propertiesDialog
{
public:
    QGridLayout *gridLayout;
    QHBoxLayout *horizontalLayout;
    QTableView *keyValueTableView;
    QVBoxLayout *verticalLayout;
    QPushButton *addKeyValuePushButton;
    QPushButton *deleteKeyValuePushButton;
    QSpacerItem *verticalSpacer;
    QDialogButtonBox *propertiesKeyValuebuttonBox;

    void setupUi(QDialog *propertiesDialog)
    {
        if (propertiesDialog->objectName().isEmpty())
            propertiesDialog->setObjectName(QStringLiteral("propertiesDialog"));
        propertiesDialog->resize(424, 330);
        gridLayout = new QGridLayout(propertiesDialog);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        keyValueTableView = new QTableView(propertiesDialog);
        keyValueTableView->setObjectName(QStringLiteral("keyValueTableView"));
        keyValueTableView->setMinimumSize(QSize(300, 250));
        keyValueTableView->setSizeIncrement(QSize(300, 200));
        keyValueTableView->setAlternatingRowColors(true);
        keyValueTableView->horizontalHeader()->setStretchLastSection(true);
        keyValueTableView->verticalHeader()->setVisible(false);

        horizontalLayout->addWidget(keyValueTableView);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        addKeyValuePushButton = new QPushButton(propertiesDialog);
        addKeyValuePushButton->setObjectName(QStringLiteral("addKeyValuePushButton"));

        verticalLayout->addWidget(addKeyValuePushButton);

        deleteKeyValuePushButton = new QPushButton(propertiesDialog);
        deleteKeyValuePushButton->setObjectName(QStringLiteral("deleteKeyValuePushButton"));

        verticalLayout->addWidget(deleteKeyValuePushButton);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);


        horizontalLayout->addLayout(verticalLayout);


        gridLayout->addLayout(horizontalLayout, 0, 0, 1, 1);

        propertiesKeyValuebuttonBox = new QDialogButtonBox(propertiesDialog);
        propertiesKeyValuebuttonBox->setObjectName(QStringLiteral("propertiesKeyValuebuttonBox"));
        propertiesKeyValuebuttonBox->setOrientation(Qt::Horizontal);
        propertiesKeyValuebuttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        gridLayout->addWidget(propertiesKeyValuebuttonBox, 1, 0, 1, 1);


        retranslateUi(propertiesDialog);
        QObject::connect(propertiesKeyValuebuttonBox, SIGNAL(accepted()), propertiesDialog, SLOT(accept()));
        QObject::connect(propertiesKeyValuebuttonBox, SIGNAL(rejected()), propertiesDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(propertiesDialog);
    } // setupUi

    void retranslateUi(QDialog *propertiesDialog)
    {
        propertiesDialog->setWindowTitle(QApplication::translate("propertiesDialog", "Dialog", Q_NULLPTR));
        addKeyValuePushButton->setText(QApplication::translate("propertiesDialog", "Add", Q_NULLPTR));
        deleteKeyValuePushButton->setText(QApplication::translate("propertiesDialog", "Delete", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class propertiesDialog: public Ui_propertiesDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // PROPERTIES_UI_H
