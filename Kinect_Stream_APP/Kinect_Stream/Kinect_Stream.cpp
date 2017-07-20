// Shiseido_Kinect_App.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "App.h"

int main()
{
	try {
		App app;
		app.init();
		app.run();
	}
	catch (exception &ex)
	{
		cout << ex.what() << endl;
		return -1;
	}
	return 0;
}