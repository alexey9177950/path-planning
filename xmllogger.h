#ifndef XMLLOGGER_H
#define	XMLLOGGER_H
#include "tinyxml2.h"
#include "ilogger.h"
#include <set>

//That's the class that flushes the data to the output XML


class XmlLogger : public ILogger {

public:
    XmlLogger(std::string loglevel):ILogger(loglevel){}

    virtual ~XmlLogger() {};

    bool getLog(const char *FileName, const std::string *LogParams);

    void saveLog();

    void writeToLogMap(const Map &Map, const std::list<Node> &path);

    void writeToLogOpenClose(const std::set<Node> &open, const std::set<Node> &close, bool last);

    void writeToLogPath(const std::list<Node> &path);

    void writeToLogHPpath(const std::list<Node> &hppath);

    void writeToLogNotFound();

    void writeToLogSummary(unsigned int numberofsteps, unsigned int nodescreated, float length, double time, double cellSize);

private:
    std::string LogFileName;
    tinyxml2::XMLDocument doc;
    int stepnum = 0;
    // make XML element for set s with tag s_name:
    tinyxml2::XMLElement *set_element(const char *s_name, const std::set<Node> &s);
};

#endif

