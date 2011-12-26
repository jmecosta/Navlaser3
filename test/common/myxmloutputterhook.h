/*
<one line to give the library's name and an idea of what it does.>
Copyright (C) 2011  <copyright holder> <email>

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef MYXMLOUTPUTTERHOOK_H
#define MYXMLOUTPUTTERHOOK_H

#include <cppunit/XmlOutputterHook.h>
#include <cppunit/tools/XmlElement.h>
#include <cppunit/tools/XmlDocument.h>
#include <cppunit/tools/StringTools.h>

#if defined(WIN32)
#include <time.h>
#endif

class MyXmlOutputterHook : public CPPUNIT_NS::XmlOutputterHook
{
public:
	MyXmlOutputterHook(const std::string projectName,
		const std::string author)
	{
		m_projectName = projectName;
		m_author      = author;
	};

	virtual ~MyXmlOutputterHook()
	{
	};

	void beginDocument(CppUnit::XmlDocument* document)
	{
		if (!document)
			return;

		// dump current time
		#if defined(WIN32)
			std::string szDate          = CPPUNIT_NS::StringTools::toString( (int)time(0) );
		#else
			std::string szDate          = CPPUNIT_NS::StringTools::toString( (int)time(0) );
		#endif

		CPPUNIT_NS::XmlElement* metaEl = new CPPUNIT_NS::XmlElement("SuiteInfo", "");

		metaEl->addElement( new CppUnit::XmlElement("Author", m_author) );
		metaEl->addElement( new CppUnit::XmlElement("Project", m_projectName) );
		metaEl->addElement( new CppUnit::XmlElement("Date", szDate ) );

		document->rootElement().addElement(metaEl);
	};
private:
	std::string m_projectName;
	std::string m_author;
};

#endif // MYXMLOUTPUTTERHOOK_H
