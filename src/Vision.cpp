#include "WPIlib.h"
#include "Vision.h"

Vision::Vision(unsigned int ResX, unsigned int ResY)
{
	Table = NetworkTable::GetTable("GRIP/myContoursReport");		// this function may change with WPIlib updates
	found = false;
	this->ResX = ResX;
	this->ResY = ResY;
	FullImageArea = this->ResX * this->ResY;
}

void Vision::Update()
{
	area = Table->GetNumberArray("area", llvm::ArrayRef<double>());
	X = Table->GetNumberArray("centerX", llvm::ArrayRef<double>());
	Y = Table->GetNumberArray("centerY", llvm::ArrayRef<double>());
	Height = Table->GetNumberArray("height", llvm::ArrayRef<double>());
	Width = Table->GetNumberArray("width", llvm::ArrayRef<double>());
}

void Vision::Filter()
{
	int NumFound = 0;
	int size = Y.size();
	int location = 0;
	for (int i = 0; i < size; i++)
	{
		if (Y.size() <= 1)
			break;
		NumFound = 0;
		for (vector<double>::iterator it = Y.begin(); it != Y.end(); it++)
		{
			if (((Y[i] >= *it) && ((Y[i] - 5) <= *it))
			|| (((Y[i] + 5) >= *it) && (Y[i] <= *it)))
			{
					NumFound++;
					int counter = 0;
					for (vector<double>::iterator locator = it; locator != Y.begin(); locator--)
						counter++;
					if(counter != i)
						location = counter;
			}
		}
		if (NumFound <= 1)
		{
			vector<double>::iterator deletiony = Y.begin();
			vector<double>::iterator deletionx = X.begin();
			vector<double>::iterator deletionheight = Height.begin();
			for (int x = 0; x < i; x++)
			{
				deletiony++;
				deletionx++;
				deletionheight++;
			}
			X.erase(deletionx);
			Y.erase(deletiony);
			Height.erase(deletionheight);
			size = Y.size();
			i--;
		}
		else if (NumFound == 2)
		{
			vector<double>::iterator Num1X = X.begin();
			vector<double>::iterator Num2X = X.begin();
			vector<double>::iterator Num1Y = Y.begin();
			vector<double>::iterator Num2Y = Y.begin();
			vector<double>::iterator Num1H = Height.begin();
			vector<double>::iterator Num2H = Height.begin();
			for (int x = 0; x < i; x++)
			{
				Num1X++;
				Num1Y++;
				Num1H++;
			}
			location -= i;
			if (location < 0)
				location *= -1;
			for (int x = 0; x < (location - i); x++)
			{
				Num2X++;
				Num2Y++;
				Num2H++;
			}

			double difference = *Num1X - *Num2X;
			if (difference < 0)
				difference *= -1;
//			if (x < 55 && x > 65)
			if (((difference < (XDIFFERENCEVAL - 5)) || difference > (XDIFFERENCEVAL + 5)))
			{
				int distance = Num1X - Num2X;
				if (distance < 0)
				{
					Y.erase(Num1Y);
					X.erase(Num1X);
					Height.erase(Num1H);
					Num2X--;
					Num2Y--;
					Num2H--;
					Y.erase(Num2Y);
					X.erase(Num2X);
					Height.erase(Num2H);
				}
				else
				{
					Y.erase(Num2Y);
					X.erase(Num2X);
					Height.erase(Num2H);
					Num1X++;
					Num1Y++;
					Num1H++;
					Y.erase(Num1Y);
					X.erase(Num1X);
					Height.erase(Num1H);
				}
				size = Y.size();
				if ((i - 2) < 0)
					i = 0;
				else
					i -= 2;
			}
		}
	}
}

void Vision::SetResolution(unsigned int x, unsigned int y)
{
	ResX = x;
	ResY = y;
	FullImageArea = ResX * ResY;
}

int Vision::GetNumContours()
{
	return Y.size();
}

double Vision::GetArea(unsigned int val)
{

	if (area.size() < val + 1)
		return 0.0;
	else
		return area[val];
}

double Vision::GetX(unsigned int val)
{

	if (X.size() < val + 1)
		return 0.0;
	else
		return X[val];
}

double Vision::GetY(unsigned int val)
{
	if (Y.size() < val + 1)
		return 0.0;
	else
		return Y[val];
}
double Vision::GetHeight(unsigned int val)
{
	if (Height.size() < val + 1)
		return 0.0;
	else
		return Height[val];
}

double Vision::GetWidth(unsigned int val)
{
	if (Width.size() < val + 1)
		return 0.0;
	else
		return Width[val];
}

double Vision::GetPercentFill(unsigned int val)
{
	if (area.size() < val + 1)
		return 0.0;
	else
		return (((Vision::GetHeight(val) * Vision::GetWidth(val)) / FullImageArea) * 100);
}

Vision::~Vision()
{}

