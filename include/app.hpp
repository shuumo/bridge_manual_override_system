/* PURPOSE:
 * wrapper that puts the other shit together because I am in a rush lol.
 * Also it is easier to delegate shit to chatgpt this way so it doesn't break
 * stuff cos it can be dumb as fuck at c++ ngl.
*/

#pragma once
#include <QApplication>
#include "mainwindow.hpp"

class App {
public:
    int run(int argc, char** argv);
};

