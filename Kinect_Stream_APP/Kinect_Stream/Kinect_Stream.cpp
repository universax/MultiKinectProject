// Shiseido_Kinect_App.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
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